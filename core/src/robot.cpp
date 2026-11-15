#include "robot.h"

#include <cmath>

#include "ranging.h"

namespace catena {

namespace {
const float kPi = 3.14159265359f;

float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }

float wrap_pi(float a) {
  const float two_pi = 6.28318530718f;
  a = std::fmod(a + kPi, two_pi);
  if (a < 0.0f) a += two_pi;
  return a - kPi;
}

uint64_t to_us(float seconds) { return static_cast<uint64_t>(seconds * 1e6f); }

// Sent in place of a turning error by a robot that cannot measure its angle yet.
const float kUnknown = 3.2767f;
bool unknown(float e) { return e > 3.25f; }

// Angles and small quantities on the radio: int16, little endian, 1e-4 units.
void put16(uint8_t* out, float value) {
  const int v = static_cast<int>(std::lround(clampf(value, -3.2f, kUnknown) * 1e4f));
  out[0] = static_cast<uint8_t>(v & 0xFF);
  out[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
}
float get16(const uint8_t* in) {
  return static_cast<float>(static_cast<int16_t>(in[0] | (in[1] << 8))) * 1e-4f;
}
bool newer(uint8_t a, uint8_t b) { return static_cast<int8_t>(a - b) > 0; }

float range_of(const Neighbour& n) { return std::hypot(n.x_mm, n.y_mm); }
float bearing_of(const Neighbour& n) { return std::atan2(n.y_mm, n.x_mm); }
}  // namespace

Robot::Robot(uint8_t id, const catena_params_t& params)
    : id_(id), p_(params), chain_(id), phase_(CATENA_WANDER_CRUISE) {
  rng_.seed(0x51ED270Bu ^ (static_cast<uint32_t>(id) * 0x01000193u));
}

bool Robot::fresh(const Neighbour& n) const {
  return n.used && now_us_ - n.seen_us <= to_us(p_.wander_memory_s);
}

bool Robot::tracked(const Neighbour& n) const {
  return n.used && now_us_ - n.seen_us <= to_us(p_.seek_memory_s);
}

bool Robot::confirmed(const Neighbour& n) const {
  return n.used && now_us_ - n.seen_us <= to_us(p_.neighbour_timeout_s);
}

Neighbour* Robot::find(uint8_t id) {
  for (Neighbour& n : neighbours_) {
    if (n.used && n.id == id) return &n;
  }
  return nullptr;
}

const Neighbour* Robot::find(uint8_t id) const { return const_cast<Robot*>(this)->find(id); }

void Robot::step(Hal& hal) {
  const uint32_t dt_us = hal.elapsed_us();
  rng_.mix(hal.entropy());
  if (!started_) {
    started_ = true;
    next_flash_us_ = static_cast<uint64_t>(rng_.uniform() * p_.flash_period_s * 1e6f);
  }
  now_us_ += dt_us;
  const float dt_s = static_cast<float>(dt_us) * 1e-6f;
  ring_rad_ = hal.ring_angle_rad();
  hall_ = hal.hall();

  // Dead reckoning: my own turn and travel move every remembered neighbour the other way.
  const float turned = hal.gyro_z_rad_s() * dt_s;
  psi_rad_ = wrap_pi(psi_rad_ + turned);
  const float moved = last_v_mm_s_ * dt_s;
  const float c = std::cos(turned);
  const float s = std::sin(turned);
  for (Neighbour& n : neighbours_) {
    if (!n.used) continue;
    const float x = n.range_mm * std::cos(n.bearing_rad) - moved;
    const float y = n.range_mm * std::sin(n.bearing_rad);
    n.bearing_rad = wrap_pi(std::atan2(y, x) - turned);
    n.range_mm = std::hypot(x, y);
    const float fx = n.x_mm - moved;
    n.x_mm = c * fx + s * n.y_mm;
    n.y_mm = c * n.y_mm - s * fx;
  }

  receive(hal);
  update_links(hal, dt_s);
  find_seeker();
  if (aft() != aft_id_) {
    aft_id_ = aft();
    aft_samples_ = 0;
  }
  push_out_x_ = push_out_y_ = 0.0f;
  has_turn_ = false;
  e_self_ = kUnknown;  // what my neighbours hear unless I hold the shape
  if (shape_ready() && !orphan_head()) {
    shape(hal, dt_s);
  } else if (chain_.ahead() != CATENA_NONE) {
    follow(hal);
  } else {
    lead(hal, dt_s);
  }
  announce(hal);

  uint8_t frame[CATENA_FRAME_MAX];
  if (chain_.behind() != CATENA_NONE) {
    // Feed-forward for the follower: my speed along the line from it to me.
    const Neighbour* b = find(chain_.behind());
    const float along = b && tracked(*b) ? -std::cos(bearing_of(*b)) : 0.0f;
    const uint8_t len = chain_.make_ack(frame, hall_, last_v_mm_s_ * along);
    put16(frame + len, e_self_);
    hal.send(chain_.behind(), frame, len + 2);
  }
  if (chain_.ahead() != CATENA_NONE) {
    // Tell the head when I can see a higher-ranked chain that it may not.
    bool sight = false;
    const Rank mine = chain_.rank();
    for (const Neighbour& n : neighbours_) {
      sight = sight || (tracked(n) && n.has_info && n.chain_id != mine.chain_id &&
                        outranks(Rank{n.size, n.chain_id}, mine));
    }
    const uint8_t len = chain_.make_up(frame, hold_here_, hall_, sight, traced_here_);
    put16(frame + len, e_self_);
    put16(frame + len + 2, wrap_pi(psi_rad_ + ring_rad_));
    hal.send(chain_.ahead(), frame, len + 4);
  }
  if (closure_ != CATENA_NONE) {
    frame[0] = CATENA_FRAME_CLOSE;
    put16(frame + 1, e_self_);
    put16(frame + 3, chain_.ahead() == CATENA_NONE ? wrap_pi(psi_rad_ + ring_rad_) : hall_);
    frame[5] = chain_.chain_id();
    hal.send(closure_, frame, 6);
  }
  shape_gossip(hal);

  catena_debug_t debug = {};
  debug.state = state_;
  if (state_ == CATENA_STATE_WANDER) {
    debug.detail = holding_ ? CATENA_WANDER_HOLD : phase_;
  } else if (state_ == CATENA_STATE_SEEK) {
    debug.detail = seek_detail_;
  } else if (state_ == CATENA_STATE_DOCK) {
    debug.detail = dock_phase_;
  } else if (state_ == CATENA_STATE_SHAPE || state_ == CATENA_STATE_MOVE) {
    debug.detail = shape_detail_;
  }
  debug.shape_version = shape_version_;
  debug.closure = closure_;
  debug.shape_error_deg = static_cast<int8_t>(
      unknown(e_self_) ? 0.0f : clampf(e_self_ * 180.0f / kPi, -127.0f, 127.0f));
  debug.index = chain_.index();
  debug.chain_size = chain_.size();
  debug.chain_id = chain_.chain_id();
  debug.ahead = chain_.ahead();
  debug.behind = chain_.behind();
  debug.target = state_ == CATENA_STATE_SEEK || state_ == CATENA_STATE_DOCK ? goal_ : CATENA_NONE;
  for (const Neighbour& n : neighbours_) {
    if (fresh(n)) ++debug.neighbours;
  }
  hal.set_debug(debug);
}

// -- radio ----------------------------------------------------------------------------------

void Robot::receive(Hal& hal) {
  for (int i = 0; i < hal.rx_count(); ++i) {
    const catena_rx_t& rx = hal.rx(i);
    if (rx.len < 1 || rx.src == id_) continue;
    const uint8_t type = rx.data[0];
    Neighbour* n = find(rx.src);

    if (type == CATENA_FRAME_FLASH) {
      FlashFix fix;
      if (rx.len >= 2 && rx.has_ir && estimate_flash(rx.ir, p_.ir_reference_range_mm, &fix)) {
        if (!n) {
          n = &neighbours_[0];
          for (Neighbour& slot : neighbours_) {
            if (!slot.used) {
              n = &slot;
              break;
            }
            if (slot.seen_us < n->seen_us) n = &slot;
          }
          *n = Neighbour{};
          n->id = rx.src;
        }
        const float x = fix.range_mm * std::cos(fix.bearing_rad);
        const float y = fix.range_mm * std::sin(fix.bearing_rad);
        const float gain = n->used && tracked(*n) ? p_.flash_filter_gain : 1.0f;
        n->x_mm += gain * (x - n->x_mm);
        n->y_mm += gain * (y - n->y_mm);
        n->bearing_rad = fix.bearing_rad;
        n->range_mm = fix.range_mm;
        n->seen_us = now_us_;
        n->used = true;
        if (rx.src == aft_id_ && aft_us_ != 0) {
          // Where the robot behind me is: its tongue direction by its own gyro and ring
          // sensor is exact but for a constant; the flash bearings pin the constant down.
          const float seen = wrap_pi(aft_tongue_rad_ + kPi - psi_rad_ - fix.bearing_rad);
          ++aft_samples_;
          const float gain = std::fmax(1.0f / static_cast<float>(aft_samples_), p_.shape_offset_gain);
          aft_offset_rad_ = wrap_pi(aft_offset_rad_ + gain * wrap_pi(seen - aft_offset_rad_));
        }
      }
      if (!n || !confirmed(*n) || rx.len < 9) continue;
      n->has_info = true;
      n->chain_id = rx.data[2];
      n->size = rx.data[3];
      n->index = rx.data[4];
      n->flags = rx.data[5];
      n->target = rx.data[6];
      n->ahead = rx.data[7];
      n->behind = rx.data[8];
      n->info_us = now_us_;
      // A head that is after my hub: tell it where it stands in my acceptance zone.
      if (n->target == id_ && chain_.behind() == CATENA_NONE && tracked(*n)) {
        const float offset = wrap_pi(bearing_of(*n) - ring_rad_ - kPi);
        const uint8_t guide[2] = {CATENA_FRAME_GUIDE,
                                  static_cast<uint8_t>(static_cast<int8_t>(offset / kPi * 127.0f))};
        hal.send(n->id, guide, sizeof guide);
      }
      continue;
    }

    // Locality: frames count only from a sender seen flashing. A linked neighbour was
    // seen when the link was made and stays accepted while the link lasts.
    const bool linked = rx.src == chain_.ahead() || rx.src == chain_.behind() ||
                        rx.src == closure_;
    if (!n || (!confirmed(*n) && !linked)) continue;
    if (type == CATENA_FRAME_LINK_REQ && rx.len >= 3) {
      const bool slot_free = req_id_ == CATENA_NONE || req_id_ == rx.src ||
                             now_us_ - req_us_ > to_us(p_.link_timeout_s);
      const bool same_chain = chain_.size() > 1 && rx.data[1] == chain_.chain_id();
      if (chain_.behind() == CATENA_NONE && slot_free && !same_chain) {
        req_id_ = rx.src;
        req_chain_ = rx.data[1];
        req_size_ = rx.data[2];
        req_us_ = now_us_;
      }
      // My own head closing a ring (spec 7.4).
      if (same_chain && chain_.behind() == CATENA_NONE && closure_ == CATENA_NONE &&
          (shape_flags_ & 3) == 3 && n->has_info && (n->flags & CATENA_FLAG_HEAD)) {
        close_req_id_ = rx.src;
        close_req_us_ = now_us_;
      }
    } else if (type == CATENA_FRAME_LINK_ACK) {
      if (chain_.ahead() == CATENA_NONE && accepts_link(*n)) {
        chain_.set_ahead(rx.src, now_us_);
        mislinked_s_ = 0.0f;
      }
      chain_.on_frame(rx.src, rx.data, rx.len, now_us_);
      if (rx.src == chain_.ahead() && rx.len >= 7) {
        e_fore_ = get16(rx.data + 5);
        e_fore_us_ = now_us_;
      }
    } else if (type == CATENA_FRAME_LINK_UP) {
      chain_.on_frame(rx.src, rx.data, rx.len, now_us_);
      if (rx.src == chain_.behind() && rx.len >= 9) {
        e_aft_ = get16(rx.data + 5);
        aft_tongue_rad_ = get16(rx.data + 7);
        aft_us_ = now_us_;
      }
    } else if (type == CATENA_FRAME_CLOSE && rx.len >= 5) {
      const bool head_end = chain_.ahead() == CATENA_NONE && chain_.behind() != CATENA_NONE;
      if (closure_ == CATENA_NONE && head_end && (shape_flags_ & 3) == 3 && n->has_info &&
          n->chain_id == chain_.chain_id() && accepts_link(*n)) {
        closure_ = rx.src;  // my tail says my tongue is on its hub
      }
      if (rx.src == closure_ && rx.len >= 6 && rx.data[5] != chain_.chain_id()) {
        // The ring lost a robot: my end and the other end are now two chains joined only by
        // this link. It becomes an ordinary link and the ring an open chain (spec 7.8).
        if (closure_split_us_ == 0) closure_split_us_ = now_us_;
        if (now_us_ - closure_split_us_ > to_us(p_.link_timeout_s)) {
          if (head_end) {
            chain_.set_ahead(closure_, now_us_);
            mislinked_s_ = 0.0f;
          } else {
            chain_.set_behind(closure_, n->has_info ? n->size : 1, rx.data[5], now_us_);
          }
          closure_ = CATENA_NONE;
          closure_split_us_ = 0;
          continue;
        }
      } else if (rx.src == closure_) {
        closure_split_us_ = 0;
      }
      if (rx.src == closure_) {
        closure_heard_us_ = now_us_;
        if (head_end) {
          e_fore_ = get16(rx.data + 1);
          e_fore_us_ = now_us_;
          closure_hall_ = get16(rx.data + 3);
        } else {
          e_aft_ = get16(rx.data + 1);
          aft_tongue_rad_ = get16(rx.data + 3);
          aft_us_ = now_us_;
        }
      }
    } else if (type == CATENA_FRAME_SHAPE && rx.len >= 9) {
      shape_frame(rx);
    } else if (type == CATENA_FRAME_GUIDE && rx.len >= 2) {
      n->guide_rad = static_cast<float>(static_cast<int8_t>(rx.data[1])) / 127.0f * kPi;
      n->guide_us = now_us_;
    } else if (type == CATENA_FRAME_LINK_NAK) {
      if (chain_.ahead() == CATENA_NONE && tracked(*n) && range_of(*n) < p_.link_range_mm &&
          phase_ != CATENA_WANDER_BACKOFF) {
        start_backoff(false, false, true);
      }
    }
  }
}

// My tongue can be on this robot's hub: it is close, and I was driving onto it or my
// tongue points at it.
bool Robot::accepts_link(const Neighbour& n) const {
  if (!tracked(n) || range_of(n) > p_.link_range_mm) return false;
  if (state_ == CATENA_STATE_DOCK && goal_ == n.id) return true;
  return std::fabs(wrap_pi(bearing_of(n) - ring_rad_)) < 0.5f * p_.wander_avoid_half_angle_rad;
}

// A free tongue, close by, inside my acceptance zone: the robot whose tip is on my hub
// when nobody asked to link.
const Neighbour* Robot::pick_adoptee() const {
  const Neighbour* best = nullptr;
  for (const Neighbour& n : neighbours_) {
    if (!tracked(n) || !n.has_info || !(n.flags & CATENA_FLAG_HEAD)) continue;
    if (n.id == refused_id_ && now_us_ < refused_until_us_) continue;
    if (range_of(n) > p_.link_range_mm) continue;
    const float off_rear = std::fabs(wrap_pi(bearing_of(n) - ring_rad_ - kPi));
    if (off_rear > p_.acceptance_half_angle_rad + p_.dock_window_rad) continue;
    if (!best || range_of(n) < range_of(*best)) best = &n;
  }
  return best;
}

void Robot::update_links(Hal& hal, float dt_s) {
  const uint64_t timeout_us = to_us(p_.link_timeout_s);
  tip_on_s_ = hall_ > p_.hall_link ? tip_on_s_ + dt_s : 0.0f;
  tip_off_s_ = hall_ < p_.hall_lost ? tip_off_s_ + dt_s : 0.0f;

  // The ring's closing link: the head's tongue on the tail's hub. It is kept apart from
  // ahead/behind, so index and count still run from head to tail.
  if (closure_ != CATENA_NONE) {
    const bool head_end = chain_.ahead() == CATENA_NONE;
    const bool an_end = head_end ? chain_.behind() != CATENA_NONE
                                 : chain_.behind() == CATENA_NONE;
    if (!an_end || now_us_ - closure_heard_us_ > timeout_us ||
        (!head_end && tip_off_s_ * p_.tick_hz > 3.0f)) {
      closure_ = CATENA_NONE;
    }
  }

  // Behind: the hall sensor says whether a tip is there; frames say whose it is.
  if (chain_.behind() != CATENA_NONE) {
    if (tip_off_s_ * p_.tick_hz > 3.0f) {
      chain_.clear_behind();
      lost_behind();
    } else if (now_us_ - chain_.behind_heard_us() > timeout_us) {
      refused_id_ = chain_.behind();
      refused_until_us_ = now_us_ + 4 * timeout_us;
      chain_.clear_behind();
      tip_on_s_ = 0.0f;
      lost_behind();
    }
  } else if (closure_ != CATENA_NONE) {
    // The tip on my hub is my own head's.
  } else if (hall_ > p_.hall_link) {
    const Neighbour* closer = close_req_id_ != CATENA_NONE && now_us_ - close_req_us_ <= timeout_us
                                  ? find(close_req_id_) : nullptr;
    const Neighbour* asked = req_id_ != CATENA_NONE && now_us_ - req_us_ <= timeout_us
                                 ? find(req_id_) : nullptr;
    if (closer && chain_.ahead() != CATENA_NONE && tracked(*closer) &&
        range_of(*closer) < p_.link_range_mm) {
      closure_ = close_req_id_;
      closure_heard_us_ = now_us_;
      close_req_id_ = CATENA_NONE;
    } else if (asked && tracked(*asked) && range_of(*asked) < p_.link_range_mm) {
      chain_.set_behind(req_id_, req_size_, req_chain_, now_us_);
      req_id_ = CATENA_NONE;
    } else if (tip_on_s_ >= p_.adopt_delay_s) {
      // A tip landed that nobody announced (robots bump into each other): offer the link.
      const Neighbour* n = pick_adoptee();
      if (n && chain_.size() > 1 && n->chain_id == chain_.chain_id()) {
        if ((shape_flags_ & 3) == 3 && chain_.ahead() != CATENA_NONE) {
          closure_ = n->id;  // my own head, and the shape is closed: the ring's last link
          closure_heard_us_ = now_us_;
        } else {
          const uint8_t nak[1] = {CATENA_FRAME_LINK_NAK};  // my own head, no ring asked for
          hal.send(n->id, nak, sizeof nak);
        }
      } else if (n) {
        chain_.set_behind(n->id, n->size, n->chain_id, now_us_);
      }
    }
  }

  // Ahead: no sensor at the tongue tip, so frames and the flash position must agree.
  if (chain_.ahead() != CATENA_NONE) {
    const Neighbour* a = find(chain_.ahead());
    bool lost = now_us_ - chain_.ahead_heard_us() > timeout_us || !a;
    if (!lost && tracked(*a)) {
      const bool off = range_of(*a) > p_.link_range_mm ||
                       std::fabs(wrap_pi(bearing_of(*a) - ring_rad_)) > p_.wander_avoid_half_angle_rad;
      mislinked_s_ = off ? mislinked_s_ + dt_s : 0.0f;
      lost = mislinked_s_ > p_.link_timeout_s;
    }
    if (lost) {
      chain_.clear_ahead();
      lost_ahead();
    }
  }
}

// Spec 7.8. A robot that loses `behind` while a shape is held becomes a tail and its chain
// stands still; one that loses `ahead` becomes a head and looks for that tail. Each waits
// `recover.search` for the other to show up and goes on waiting while it is in sight, at
// most `recover.limit`.
void Robot::lost_behind() {
  if (!shape_wanted()) return;
  orphan_tail_until_us_ = now_us_ + to_us(p_.recover_search_s);
  orphan_tail_limit_us_ = now_us_ + to_us(p_.recover_limit_s);
}

void Robot::lost_ahead() {
  if (!shape_wanted()) return;
  orphan_head_until_us_ = now_us_ + to_us(p_.recover_search_s);
  orphan_head_limit_us_ = now_us_ + to_us(p_.recover_limit_s);
}

// The tail that lost the robot between us, if it is in sight.
const Neighbour* Robot::orphan_goal() const {
  const Neighbour* best = nullptr;
  for (const Neighbour& n : neighbours_) {
    if (!tracked(n) || !n.has_info || n.chain_id == chain_.chain_id()) continue;
    if ((n.flags & (CATENA_FLAG_ORPHAN | CATENA_FLAG_TAIL)) != (CATENA_FLAG_ORPHAN | CATENA_FLAG_TAIL)) continue;
    if (!best || range_of(n) < range_of(*best)) best = &n;
  }
  return best;
}

// A lower-ranked head close by that announces my hub as its target: the chain stands still
// for it.
void Robot::find_seeker() {
  hold_here_ = false;
  seeker_ = CATENA_NONE;
  traced_here_ = false;
  for (const Neighbour& n : neighbours_) {
    traced_here_ = traced_here_ ||
                   (tracked(n) && n.has_info && (n.flags & CATENA_FLAG_HEAD) &&
                    n.target != CATENA_NONE && n.chain_id != chain_.chain_id() &&
                    now_us_ - n.info_us <= to_us(p_.wander_memory_s) &&
                    outranks(chain_.rank(), Rank{n.size, n.chain_id}));
  }
  // The other end of a gap left by a removed robot: keep waiting while it is in sight.
  // If it turns out to be in my own chain (a ring that lost a robot), the wait is over.
  for (const Neighbour& n : neighbours_) {
    if (!tracked(n) || !n.has_info || !(n.flags & CATENA_FLAG_ORPHAN)) continue;
    if (now_us_ - n.info_us > to_us(p_.wander_memory_s)) continue;
    const bool same = n.chain_id == chain_.chain_id();
    const uint64_t more = now_us_ + to_us(p_.recover_search_s);
    if ((n.flags & CATENA_FLAG_HEAD) && orphan_tail()) {
      orphan_tail_until_us_ = same ? 0 : (more < orphan_tail_limit_us_ ? more : orphan_tail_limit_us_);
    }
    if ((n.flags & CATENA_FLAG_TAIL) && orphan_head()) {
      orphan_head_until_us_ = same ? 0 : (more < orphan_head_limit_us_ ? more : orphan_head_limit_us_);
    }
  }
  if (chain_.behind() != CATENA_NONE || hall_ > p_.hall_link) return;
  hold_here_ = orphan_tail();  // my chain stands still for the head that lost us
  const Rank mine = chain_.rank();
  float best_mm = p_.seek_hold_range_mm;
  for (const Neighbour& n : neighbours_) {
    if (!tracked(n) || !n.has_info || n.target != id_ || !(n.flags & CATENA_FLAG_HEAD)) continue;
    if (now_us_ - n.info_us > to_us(p_.wander_memory_s)) continue;
    if (!outranks(mine, Rank{n.size, n.chain_id}) && !orphan_tail()) continue;
    if (range_of(n) < best_mm) {
      best_mm = range_of(n);
      seeker_ = n.id;
      hold_here_ = true;
    }
  }
}

void Robot::announce(Hal& hal) {
  const uint64_t tick_us = static_cast<uint64_t>(1e6f / p_.tick_hz);
  if (next_flash_us_ >= now_us_ + tick_us) return;
  const uint32_t delay_us =
      next_flash_us_ > now_us_ ? static_cast<uint32_t>(next_flash_us_ - now_us_) : 0u;
  uint8_t flags = 0;
  if (fore() == CATENA_NONE) flags |= CATENA_FLAG_HEAD;
  if (chain_.behind() == CATENA_NONE && hall_ <= p_.hall_link) flags |= CATENA_FLAG_TAIL;
  if (state_ == CATENA_STATE_DOCK) flags |= CATENA_FLAG_DOCKING;
  if (((flags & CATENA_FLAG_TAIL) && orphan_tail()) || ((flags & CATENA_FLAG_HEAD) && orphan_head())) {
    flags |= CATENA_FLAG_ORPHAN;
  }
  const uint8_t frame[9] = {CATENA_FRAME_FLASH, id_, chain_.chain_id(), chain_.size(),
                            chain_.index(), flags, target_, chain_.ahead(), chain_.behind()};
  hal.flash(frame, sizeof frame, delay_us);
  const float period_us =
      p_.flash_period_s * 1e6f * (1.0f + p_.flash_jitter_frac * rng_.uniform(-1.0f, 1.0f));
  next_flash_us_ += static_cast<uint64_t>(period_us);
  if (next_flash_us_ < now_us_) next_flash_us_ = now_us_ + static_cast<uint64_t>(period_us);
}

// -- linked: FOLLOW -------------------------------------------------------------------------

// Train behind the robot ahead. The tongue points at its hub, so the ring angle is the
// bearing; the gap comes from the hall reading it reports. Both front sensors over the
// edge stop the wheels; one alone does not, because the hub ahead stands on the table.
void Robot::follow(Hal& hal) {
  state_ = CATENA_STATE_FOLLOW;
  holding_ = false;
  goal_ = target_ = CATENA_NONE;
  phase_ = CATENA_WANDER_CRUISE;
  // No feed-forward backwards: there are no edge sensors at the rear.
  float v = std::fmax(chain_.ahead_speed_mm_s(), 0.0f) +
            p_.follow_gain_mm_s * (p_.follow_hall_target - chain_.ahead_hall());
  v *= clampf(std::cos(ring_rad_), 0.0f, 1.0f);
  if (!hal.table_left() && !hal.table_right() && v > 0.0f) v = 0.0f;
  drive(hal, v, p_.follow_turn_gain_per_s * ring_rad_);
}

// -- SHAPE and MOVE (spec 7.4, 7.5) ---------------------------------------------------------

uint8_t Robot::fore() const {
  if (chain_.ahead() != CATENA_NONE) return chain_.ahead();
  return chain_.behind() != CATENA_NONE ? closure_ : CATENA_NONE;
}

uint8_t Robot::aft() const {
  if (chain_.behind() != CATENA_NONE) return chain_.behind();
  return chain_.ahead() != CATENA_NONE ? closure_ : CATENA_NONE;
}

void Robot::set_shape(const uint8_t* xy, bool closed, bool active) {
  for (int k = 0; k < 2 * kStrokePoints; ++k) stroke_.xy[k] = xy[k];
  ++shape_version_;
  if (shape_version_ == 0) shape_version_ = 1;
  shape_flags_ = static_cast<uint8_t>((closed ? 1 : 0) | (active ? 2 : 0));
  chunks_ = 0x0F;
}

void Robot::set_twist(float vx_mm_s, float vy_mm_s, float wz_rad_s) {
  ++twist_seq_;
  twist_[0] = static_cast<int8_t>(clampf(vx_mm_s, -127.0f, 127.0f));
  twist_[1] = static_cast<int8_t>(clampf(vy_mm_s, -127.0f, 127.0f));
  twist_[2] = static_cast<int8_t>(clampf(wz_rad_s * 100.0f, -127.0f, 127.0f));
}

// A SHAPE frame from a linked neighbour: the newest shape and the newest twist win.
void Robot::shape_frame(const catena_rx_t& rx) {
  const uint8_t version = rx.data[1];
  if (version == 0) return;
  if (shape_version_ == 0 || newer(version, shape_version_)) {
    shape_version_ = version;
    shape_flags_ = rx.data[2] & 3;
    chunks_ = 0;
  }
  if (version != shape_version_) return;
  if (rx.len >= 9 + 16) {
    const int chunk = (rx.data[2] >> 4) & 3;
    for (int k = 0; k < 16; ++k) stroke_.xy[16 * chunk + k] = rx.data[9 + k];
    chunks_ |= static_cast<uint8_t>(1u << chunk);
  }
  if (newer(rx.data[3], twist_seq_)) {
    twist_seq_ = rx.data[3];
    for (int k = 0; k < 3; ++k) twist_[k] = static_cast<int8_t>(rx.data[4 + k]);
  }
  const float px = static_cast<float>(static_cast<int8_t>(rx.data[7]));
  const float py = static_cast<float>(static_cast<int8_t>(rx.data[8]));
  const bool stale = now_us_ - push_in_us_ > to_us(p_.link_timeout_s);
  if (stale || std::hypot(px, py) >= std::hypot(push_in_x_, push_in_y_)) {
    push_in_x_ = px;
    push_in_y_ = py;
    push_in_us_ = now_us_;
  }
}

// One SHAPE frame per step, to the robot ahead and the robot behind in turn, a quarter of
// the stroke at a time.
void Robot::shape_gossip(Hal& hal) {
  if (shape_version_ == 0) return;
  ++gossip_count_;
  const uint8_t to = gossip_count_ & 1 ? fore() : aft();
  if (to == CATENA_NONE) return;
  const int chunk = (gossip_count_ >> 1) & 3;
  uint8_t frame[9 + 16];
  frame[0] = CATENA_FRAME_SHAPE;
  frame[1] = shape_version_;
  frame[2] = static_cast<uint8_t>(shape_flags_ | (chunk << 4));
  frame[3] = twist_seq_;
  for (int k = 0; k < 3; ++k) frame[4 + k] = static_cast<uint8_t>(twist_[k]);
  frame[7] = static_cast<uint8_t>(static_cast<int8_t>(clampf(push_out_x_, -127.0f, 127.0f)));
  frame[8] = static_cast<uint8_t>(static_cast<int8_t>(clampf(push_out_y_, -127.0f, 127.0f)));
  const bool have = (chunks_ >> chunk) & 1;
  for (int k = 0; k < 16; ++k) frame[9 + k] = stroke_.xy[16 * chunk + k];
  hal.send(to, frame, have ? sizeof frame : 9);
}

// True while this robot holds a shape: an active stroke has arrived in full and the chain
// is long enough. Rebuilds the polygon when the stroke or the chain's size changes.
bool Robot::shape_ready() {
  const int n = chain_.size();
  if (shape_version_ == 0 || !(shape_flags_ & 2) || chunks_ != 0x0F || n < 2) return false;
  if (poly_version_ != shape_version_ || poly_.n != n) {
    stroke_.closed = (shape_flags_ & 1) != 0;
    poly_ok_ = make_polygon(stroke_, n, p_.pitch_mm, p_.max_joint_yaw_rad, &poly_);
    poly_version_ = shape_version_;
    poly_.n = n;
  }
  return poly_ok_ && chain_.index() < n;
}

const Neighbour* Robot::own_tail() const {
  for (const Neighbour& n : neighbours_) {
    if (tracked(n) && n.has_info && n.chain_id == chain_.chain_id() && n.id != chain_.behind() &&
        (n.flags & CATENA_FLAG_TAIL) && n.size == chain_.size()) {
      return &n;
    }
  }
  return nullptr;
}

void Robot::shape(Hal& hal, float dt_s) {
  holding_ = false;
  goal_ = target_ = CATENA_NONE;
  phase_ = CATENA_WANDER_CRUISE;
  shape_detail_ = CATENA_SHAPE_HOLD;
  const bool moving = twist_[0] != 0 || twist_[1] != 0 || twist_[2] != 0;
  state_ = moving ? CATENA_STATE_MOVE : CATENA_STATE_SHAPE;

  const int n = poly_.n;
  const int i = chain_.index();
  const uint8_t f = fore();
  const bool head_end = chain_.ahead() == CATENA_NONE;
  const uint64_t stale_us = to_us(p_.link_timeout_s);
  const float e_fore = now_us_ - e_fore_us_ <= stale_us ? e_fore_ : 0.0f;
  if (chain_.ahead() != CATENA_NONE && unknown(e_fore)) {
    // The robot ahead does not hold the shape yet (the stroke has not reached it, or it
    // cannot measure its angle): keep following it. The shape takes hold from the head.
    follow(hal);
    return;
  }
  e_self_ = 0.0f;
  if (hold_here_ || chain_.hold_behind()) {
    // A robot is docking onto my chain's tail, or the tail waits for the half that was
    // cut off: stand still. The shape for the new count comes after the link.
    drive(hal, 0.0f, 0.0f);
    return;
  }
  const bool has_aft = aft_id_ != CATENA_NONE && now_us_ - aft_us_ <= stale_us &&
                       static_cast<float>(aft_samples_) >= p_.shape_offset_samples;
  const float aft_rad = wrap_pi(aft_tongue_rad_ + kPi - psi_rad_ - aft_offset_rad_);
  // u: the link from the robot ahead to me. w: the link from me to the robot behind.
  const float ux = -std::cos(ring_rad_), uy = -std::sin(ring_rad_);
  const float wx = std::cos(aft_rad), wy = std::sin(aft_rad);

  // Shape frame to base frame: the polygon says where my edge points, the ring sensor
  // (or, for the head, the robot behind) says where it points for me.
  const bool framed = f != CATENA_NONE || has_aft;
  const float rho = f != CATENA_NONE ? wrap_pi(ring_rad_ + kPi - poly_.edge_rad[i])
                                     : wrap_pi(aft_rad - poly_.edge_rad[1]);
  const float cr = std::cos(rho), sr = std::sin(rho);

  // Edge push: mine while a sensor is over the edge, else what a neighbour passes on.
  const bool at_edge = !hal.table_left() || !hal.table_right();
  if (at_edge && framed) {
    // The edge lies where my front sensors see the void: ahead, or ahead and to one side.
    // I may have met it at a slant or been pushed onto it sideways, so the direction is
    // averaged over every heading at which the void shows while I turn.
    const float side = hal.table_left() ? -0.5f : (hal.table_right() ? 0.5f : 0.0f);
    const float toward = side - rho;  // direction of the void in the shape frame
    const float keep = now_us_ >= edge_until_us_ ? 0.0f : 0.8f;
    float ex = keep * -edge_push_x_ / p_.shape_edge_push_mm_s + (1.0f - keep) * std::cos(toward);
    float ey = keep * -edge_push_y_ / p_.shape_edge_push_mm_s + (1.0f - keep) * std::sin(toward);
    const float length = std::hypot(ex, ey);
    if (length > 1e-3f) {
      edge_push_x_ = -p_.shape_edge_push_mm_s * ex / length;
      edge_push_y_ = -p_.shape_edge_push_mm_s * ey / length;
    }
    edge_until_us_ = now_us_ + to_us(p_.shape_edge_hold_s);
  }
  if (now_us_ < edge_until_us_) {
    push_out_x_ = edge_push_x_;
    push_out_y_ = edge_push_y_;
  } else if (now_us_ - push_in_us_ <= stale_us) {
    push_out_x_ = p_.shape_push_decay * push_in_x_;
    push_out_y_ = p_.shape_push_decay * push_in_y_;
  }
  // Never forward over a void, and never backward either: there is no sensor at the rear,
  // and a robot parallel to an edge would reverse along it (it did, and fell). The base
  // keeps turning toward its velocity, which the push now points away from the edge.
  if (at_edge) shape_detail_ = CATENA_SHAPE_EDGE;

  const bool ring_wanted = poly_.closed && n >= 3;
  if (closure_ != CATENA_NONE && !ring_wanted && head_end && !at_edge) {
    // The new shape is open: line the base up with the tongue and pull it out.
    shape_detail_ = CATENA_SHAPE_OPENING;
    const bool lined_up = std::fabs(ring_rad_) < p_.dock_align_rad;
    drive(hal, lined_up ? -p_.dock_speed_mm_s : 0.0f,
          clampf(p_.shape_turn_gain_per_s * ring_rad_, -p_.wander_spin_rate_rad_s,
                 p_.wander_spin_rate_rad_s));
    return;
  }
  if (ring_wanted && head_end && closure_ == CATENA_NONE && !at_edge) {
    // A closed shape closes itself: the head docks onto its own tail once it is near.
    // ... and lies where the shape puts it: ends that only pass each other do not link.
    const Neighbour* tail = own_tail();
    const float expected = rho + std::atan2(poly_.y_mm[n - 1] - poly_.y_mm[0],
                                            poly_.x_mm[n - 1] - poly_.x_mm[0]);
    if (tail && framed && range_of(*tail) < p_.shape_close_range_mm &&
        std::fabs(wrap_pi(bearing_of(*tail) - expected)) < p_.shape_close_cone_rad) {
      shape_detail_ = CATENA_SHAPE_CLOSING;
      target_ = tail->id;
      const float bearing = bearing_of(*tail);
      const float w = p_.seek_turn_gain_per_s * bearing;
      if (std::fabs(bearing) < p_.dock_align_rad && std::fabs(ring_rad_) < p_.dock_align_rad) {
        const uint8_t request[3] = {CATENA_FRAME_LINK_REQ, chain_.chain_id(), chain_.size()};
        hal.send(tail->id, request, sizeof request);
        // No faster than the robot on my hub keeps up: driving in must not tear it off.
        const float keep = clampf((hall_ - p_.follow_slack_stop) /
                                  (p_.follow_slack_full - p_.follow_slack_stop), 0.0f, 1.0f);
        drive(hal, keep * p_.dock_speed_mm_s,
              clampf(w, -p_.dock_max_turn_rate_rad_s, p_.dock_max_turn_rate_rad_s));
      } else {
        drive(hal, 0.0f, clampf(w, -p_.wander_spin_rate_rad_s, p_.wander_spin_rate_rad_s));
      }
      return;
    }
  }

  // The local rule (spec 7.4): descend the sum of squared turning errors, using my own two
  // links and my neighbours' errors.
  const bool opening = closure_ != CATENA_NONE && !ring_wanted;
  has_turn_ = f != CATENA_NONE && has_aft && !opening;
  e_self_ = has_turn_ ? wrap_pi(std::atan2(ux * wy - uy * wx, ux * wx + uy * wy) - poly_.turn_rad[i])
                      : 0.0f;
  const float e_aft = now_us_ - aft_us_ <= stale_us ? e_aft_ : 0.0f;
  // A robot between two others that does not know its angle yet says so, and it and its
  // neighbours wait: a made-up zero would send them all off against each other.
  const bool blind = f != CATENA_NONE && aft_id_ != CATENA_NONE && !has_aft;
  if (blind) e_self_ = kUnknown;
  const bool wait = blind || unknown(e_fore) || unknown(e_aft);
  const float g = p_.shape_gain_per_s * p_.pitch_mm;
  float vx = 0.0f, vy = 0.0f;
  if (f != CATENA_NONE && !wait) {
    vx -= g * (e_fore - e_self_) * -uy;
    vy -= g * (e_fore - e_self_) * ux;
  }
  if (has_aft && !wait) {
    vx -= g * (e_aft - e_self_) * -wy;
    vy -= g * (e_aft - e_self_) * wx;
  }
  const float correction = std::hypot(vx, vy);
  if (correction > p_.shape_max_speed_mm_s) {
    vx *= p_.shape_max_speed_mm_s / correction;
    vy *= p_.shape_max_speed_mm_s / correction;
  }
  // Close a stretched gap to the hub ahead, as in FOLLOW: the magnet alone holds only so
  // much. Never open one: that would turn the base right round, again and again.
  if (f != CATENA_NONE) {
    const float hall_ahead = chain_.ahead() != CATENA_NONE ? chain_.ahead_hall() : closure_hall_;
    const float gap = clampf(p_.follow_gain_mm_s * (p_.follow_hall_target - hall_ahead),
                             0.0f, p_.follow_lead_speed_mm_s);
    vx += gap * -ux;
    vy += gap * -uy;
  }
  if (has_aft) {
    // The same for the link on my own hub: give way toward the robot behind.
    const float gap = clampf(p_.follow_gain_mm_s * (p_.follow_hall_target - hall_), 0.0f,
                             p_.follow_lead_speed_mm_s);
    vx += gap * wx;
    vy += gap * wy;
  }
  // Twist and edge push are given in the shape frame (spec 7.5).
  if (framed) {
    float tx = static_cast<float>(twist_[0]), ty = static_cast<float>(twist_[1]);
    const float speed = std::hypot(tx, ty);
    if (speed > p_.shape_twist_max_speed_mm_s) {
      tx *= p_.shape_twist_max_speed_mm_s / speed;
      ty *= p_.shape_twist_max_speed_mm_s / speed;
    }
    const float wz = clampf(static_cast<float>(twist_[2]) * 0.01f, -p_.shape_twist_max_rate_rad_s,
                            p_.shape_twist_max_rate_rad_s);
    // Travel: turning the base toward the motion costs time, so each robot keeps count of
    // the distance it owes along the commanded direction and makes it up.
    const float want = std::hypot(tx, ty);
    if (want > 0.0f) {
      owed_x_ = tx / want;
      owed_y_ = ty / want;
    }
    const float driven = last_v_mm_s_ * (cr * owed_x_ - sr * owed_y_);  // along it, last step
    if (want > 0.0f || owed_mm_ != 0.0f) {
      owed_mm_ = clampf(owed_mm_ + (want - driven) * dt_s, -p_.pitch_mm, p_.pitch_mm);
    }
    if (want == 0.0f && std::fabs(owed_mm_) < 1.0f) owed_mm_ = 0.0f;
    // No faster than a shape correction: robots at different speeds tear links apart.
    const float catchup = clampf(p_.shape_twist_catchup_per_s * owed_mm_,
                                 -p_.shape_max_speed_mm_s, p_.shape_max_speed_mm_s);
    if (owed_mm_ != 0.0f) state_ = CATENA_STATE_MOVE;
    // Set off gently and no faster than my links keep up: some bases are still turning
    // into the motion while others already drive.
    ramp_ = want > 0.0f ? std::fmin(1.0f, ramp_ + dt_s / p_.shape_twist_ramp_s) : 0.0f;
    float slack = f != CATENA_NONE
        ? (chain_.ahead() != CATENA_NONE ? chain_.ahead_hall() : closure_hall_) : 1.0f;
    if (aft_id_ != CATENA_NONE) slack = std::fmin(slack, hall_);
    const float keep = clampf((slack - p_.follow_slack_stop) /
                              (p_.follow_slack_full - p_.follow_slack_stop), 0.0f, 1.0f);
    const float along = keep * (ramp_ * want + catchup);
    const float sx = along * owed_x_ - wz * poly_.y_mm[i] + push_out_x_;
    const float sy = along * owed_y_ + wz * poly_.x_mm[i] + push_out_y_;
    vx += cr * sx - sr * sy;
    vy += sr * sx + cr * sy;
  }
  shape_drive(hal, vx, vy, !at_edge);
}

// The base turns toward the velocity and drives, slower while misaligned (spec 7.5).
void Robot::shape_drive(Hal& hal, float vx_mm_s, float vy_mm_s, bool forward_ok) {
  const float speed = std::hypot(vx_mm_s, vy_mm_s);
  if (speed < p_.shape_min_speed_mm_s) {
    drive(hal, 0.0f, 0.0f);
    return;
  }
  const float error = std::atan2(vy_mm_s, vx_mm_s);
  const float forward = clampf(std::cos(error), 0.0f, 1.0f);
  drive(hal, forward_ok ? speed * forward * forward : 0.0f,
        clampf(p_.shape_turn_gain_per_s * error, -p_.wander_spin_rate_rad_s,
               p_.wander_spin_rate_rad_s));
}

// -- head: WANDER, SEEK, DOCK ---------------------------------------------------------------

// A head with robots behind it slows down when any link of its chain stretches.
float Robot::lead_speed_mm_s() const {
  const float span = p_.follow_slack_full - p_.follow_slack_stop;
  // Slower while a lower-ranked head works along my chain: it tows too and must catch up.
  const float traced = traced_here_ || chain_.traced_behind() ? p_.seek_traced_speed : 1.0f;
  return traced * p_.follow_lead_speed_mm_s *
         clampf((chain_.slack(hall_) - p_.follow_slack_stop) / span, 0.0f, 1.0f);
}

void Robot::lead(Hal& hal, float dt_s) {
  target_ = CATENA_NONE;
  holding_ = false;
  const bool at_edge = !hal.table_left() || !hal.table_right();
  if (at_edge || phase_ == CATENA_WANDER_BACKOFF || phase_ == CATENA_WANDER_TURN) {
    state_ = CATENA_STATE_WANDER;  // edge avoidance overrides seeking and docking
    goal_ = CATENA_NONE;
    wander(hal, dt_s);
    return;
  }
  if (hold_here_ || chain_.hold_behind()) {
    // A robot is docking onto the chain's tail: stand still. Alone, I am that tail and
    // turn my back to it, which brings my acceptance zone round.
    state_ = CATENA_STATE_WANDER;
    holding_ = true;
    goal_ = CATENA_NONE;
    float w = 0.0f;
    const Neighbour* s = find(seeker_);
    if (!towing() && s && tracked(*s)) {
      w = clampf(p_.seek_turn_gain_per_s * wrap_pi(bearing_of(*s) - kPi),
                 -p_.wander_spin_rate_rad_s, p_.wander_spin_rate_rad_s);
    }
    drive(hal, 0.0f, w);
    return;
  }
  if (orphan_head()) {
    // I lost the robot ahead while holding a shape: go for the tail it left, or wait for
    // it to show. Rank does not matter here, and I do not wander off with my half.
    const Neighbour* tail = orphan_goal();
    if (!tail) {
      state_ = CATENA_STATE_WANDER;
      holding_ = true;
      goal_ = CATENA_NONE;
      drive(hal, 0.0f, 0.0f);
      return;
    }
    seek(hal, *tail, dt_s);
    return;
  }
  const Neighbour* goal = pick_goal();
  if (!goal) {
    state_ = CATENA_STATE_WANDER;
    goal_ = CATENA_NONE;
    wander(hal, dt_s);
    return;
  }
  seek(hal, *goal, dt_s);
}

// The nearest robot of a higher-ranked chain, moved along that chain towards its tail as
// far as its members are in sight.
const Neighbour* Robot::pick_goal() const {
  const Rank mine = chain_.rank();
  const Neighbour* best = nullptr;
  float best_cost = 0.0f;
  for (const Neighbour& n : neighbours_) {
    if (!tracked(n) || !n.has_info || n.chain_id == mine.chain_id) continue;
    if (!outranks(Rank{n.size, n.chain_id}, mine)) continue;
    const Neighbour* g = &n;
    for (int hop = 0; hop < CATENA_DESIGN_COUNT && !(g->flags & CATENA_FLAG_TAIL); ++hop) {
      const Neighbour* next = g->behind != CATENA_NONE ? find(g->behind) : nullptr;
      if (!next || !tracked(*next) || !next->has_info) break;
      g = next;
    }
    // A tail beats any other member, and further down the chain beats nearer the head.
    const float along = g->flags & CATENA_FLAG_TAIL ? CATENA_DESIGN_COUNT : g->index;
    const float cost = range_of(*g) * (g->id == goal_ ? p_.seek_keep_ratio : 1.0f) -
                       along * p_.ir_range_mm;
    if (!best || cost < best_cost) {
      best = g;
      best_cost = cost;
    }
  }
  if (best || !chain_.sight_behind() || p_.seek_turn_back < 0.5f) return best;
  // A robot further back sees a higher-ranked chain: go back along my own chain, towards
  // the member furthest down that is in sight, until I see that chain myself.
  for (const Neighbour& n : neighbours_) {
    if (!tracked(n) || !n.has_info || n.chain_id != mine.chain_id || n.id == chain_.behind()) continue;
    if (!best || n.index > best->index) best = &n;
  }
  return best;
}

// With a robot on my hub my tongue reaches only directions far enough from it (the joint's
// yaw limit). True when the tongue can point along bearing_rad.
bool Robot::tow_clear(float bearing_rad) const {
  const Neighbour* b = towing() ? find(chain_.behind()) : nullptr;
  if (!b || !tracked(*b)) return true;
  return std::fabs(wrap_pi(bearing_rad - bearing_of(*b))) >
         kPi - p_.max_joint_yaw_rad + p_.dock_tow_margin_rad;
}

// Another head is lining up on the same tail and goes first: it started before me, or we
// both started and it has the lower id.
bool Robot::rival(const Neighbour& goal) const {
  for (const Neighbour& n : neighbours_) {
    if (!n.used || !n.has_info || n.id == goal.id || n.target != goal.id) continue;
    if (!(n.flags & CATENA_FLAG_DOCKING) || now_us_ - n.info_us > to_us(p_.wander_memory_s)) continue;
    if (state_ != CATENA_STATE_DOCK || n.id < id_) return true;
  }
  return false;
}

void Robot::seek(Hal& hal, const Neighbour& goal, float dt_s) {
  if (goal.id != goal_) {
    goal_ = goal.id;
    state_ = CATENA_STATE_SEEK;
    attempts_ = 0;
    leave_left_mm_ = 0.0f;
    approach_bias_rad_ = 0.0f;
  }
  const bool tail = (goal.flags & CATENA_FLAG_TAIL) != 0 && goal.chain_id != chain_.chain_id();
  const float range_mm = range_of(goal);
  const float bearing = bearing_of(goal);
  const bool guided = tail && goal.guide_us != 0 && now_us_ - goal.guide_us <= to_us(p_.seek_memory_s);
  const float offset = guided ? wrap_pi(goal.guide_rad - approach_bias_rad_) : 0.0f;
  const bool wait = rival(goal);
  const float speed = towing() ? lead_speed_mm_s() : p_.seek_speed_mm_s;

  // Steer round every other robot in the way.
  float push_x = 0.0f;
  float push_y = 0.0f;
  for (const Neighbour& n : neighbours_) {
    if (!fresh(n) || n.id == goal.id || n.id == chain_.behind()) continue;
    const float r = range_of(n);
    if (r >= p_.wander_avoid_range_mm || r < 1.0f) continue;
    const float k = clampf((p_.wander_avoid_range_mm - r) /
                               (p_.wander_avoid_range_mm - p_.wander_avoid_stop_range_mm), 0.0f, 2.0f);
    push_x -= k * n.x_mm / r;
    push_y -= k * n.y_mm / r;
  }

  if (leave_left_mm_ > 0.0f) {
    // Three failed attempts: get away from the tail, then come back from another side.
    state_ = CATENA_STATE_SEEK;
    seek_detail_ = CATENA_SEEK_LEAVE;
    leave_left_mm_ -= std::fabs(last_v_mm_s_) * dt_s + 0.1f * speed * dt_s;
    steer(hal, push_x - std::cos(bearing), push_y - std::sin(bearing), speed, bearing + kPi);
    return;
  }
  const bool own = goal.chain_id == chain_.chain_id();
  if (tail) target_ = goal.id;
  if (state_ == CATENA_STATE_DOCK && tail && guided) {
    dock(hal, goal, range_mm, bearing, offset, wait, dt_s);
    return;
  }
  state_ = CATENA_STATE_SEEK;
  seek_detail_ = tail ? CATENA_SEEK_TAIL : CATENA_SEEK_TRACE;
  // A robot towed round the standoff circle trails inside it, where the joint on my hub
  // stops my tongue short of the tail. So with a chain I walk round further out and then
  // run straight in, which pulls the chain in behind me.
  const bool tow = tail && towing();
  const bool clear = tow_clear(bearing);
  const float outer = p_.seek_standoff_mm + p_.seek_tow_standoff_extra_mm;
  if (!tow || (guided && std::fabs(offset) > p_.dock_window_rad)) {
    run_in_ = false;
  } else if (guided && std::fabs(offset) < 0.5f * p_.dock_window_rad &&
             (clear || range_mm > outer - 0.25f * p_.seek_tow_standoff_extra_mm)) {
    run_in_ = true;
  }
  const float standoff = (tow && !run_in_ ? outer : p_.seek_standoff_mm) +
                         (wait ? 0.5f * p_.pitch_mm : 0.0f);
  if (guided && !wait && range_mm < standoff + 0.25f * p_.pitch_mm &&
      std::fabs(offset) < p_.dock_window_rad && clear) {
    state_ = CATENA_STATE_DOCK;
    dock_phase_ = CATENA_DOCK_ALIGN;
    phase_start_us_ = now_us_;
    drive(hal, 0.0f, 0.0f);
    return;
  }
  // Close in to the standoff circle and walk round it into the acceptance zone.
  const float radial = clampf((range_mm - standoff) / (0.5f * p_.pitch_mm), tow ? -1.0f : 0.0f, 1.0f);
  // Round my own tail I keep walking: the chain a robot back there sees is somewhere near.
  const float round = own ? 1.0f : guided ? clampf(offset / p_.dock_window_rad, -1.0f, 1.0f) : 0.0f;
  const float dx = radial * std::cos(bearing) - round * std::sin(bearing) + push_x;
  const float dy = radial * std::sin(bearing) + round * std::cos(bearing) + push_y;
  steer(hal, dx, dy, speed, bearing);
}

// Final approach, tongue first (spec 7.2): line up, send LINK_REQ and drive straight in;
// the link exists once the tail answers LINK_ACK (handled in receive()).
void Robot::dock(Hal& hal, const Neighbour& goal, float range_mm, float bearing_rad,
                 float offset_rad, bool wait, float dt_s) {
  const bool timed_out = now_us_ - phase_start_us_ > to_us(p_.dock_timeout_s);
  const float w = p_.seek_turn_gain_per_s * bearing_rad;
  if (dock_phase_ == CATENA_DOCK_ALIGN) {
    if (wait || timed_out || std::fabs(offset_rad) > 2.0f * p_.dock_window_rad ||
        range_mm > p_.seek_standoff_mm + 0.5f * p_.pitch_mm) {
      state_ = CATENA_STATE_SEEK;
      drive(hal, 0.0f, 0.0f);
      return;
    }
    drive(hal, 0.0f, clampf(w, -p_.wander_spin_rate_rad_s, p_.wander_spin_rate_rad_s));
    if (std::fabs(bearing_rad) < p_.dock_align_rad && std::fabs(ring_rad_) < p_.dock_align_rad) {
      dock_phase_ = CATENA_DOCK_DRIVE;
      phase_start_us_ = now_us_;
      driven_mm_ = range_mm;  // where the drive started
    }
  } else if (dock_phase_ == CATENA_DOCK_DRIVE) {
    const uint8_t request[3] = {CATENA_FRAME_LINK_REQ, chain_.chain_id(), chain_.size()};
    hal.send(goal.id, request, sizeof request);
    drive(hal, p_.dock_speed_mm_s,
          clampf(w, -p_.dock_max_turn_rate_rad_s, p_.dock_max_turn_rate_rad_s));
    if (wait || timed_out || std::fabs(bearing_rad) > 4.0f * p_.dock_align_rad) {
      if (!wait) ++attempts_;
      dock_phase_ = CATENA_DOCK_RETREAT;
      // Wheels slip against a blocked tail, so the range tells how far I really came.
      driven_mm_ = clampf(driven_mm_ - range_mm, 0.0f, 0.75f * p_.pitch_mm);
    }
  } else {
    // Back out along the line just driven: the only stretch behind me known to be table.
    driven_mm_ -= p_.dock_speed_mm_s * dt_s;
    drive(hal, -p_.dock_speed_mm_s, 0.0f);
    if (driven_mm_ <= 0.0f) {
      state_ = CATENA_STATE_SEEK;
      if (attempts_ >= static_cast<int>(p_.dock_attempts)) {
        attempts_ = 0;
        leave_left_mm_ = p_.dock_backoff_distance_mm;
        approach_bias_rad_ = rng_.uniform(-1.0f, 1.0f) * p_.dock_window_rad;
      }
    }
  }
}

// Drive along (dx, dy) in the base frame; its length, up to 1, scales the speed. With
// nowhere to go, face rest_bearing_rad.
void Robot::steer(Hal& hal, float dx, float dy, float speed_mm_s, float rest_bearing_rad) {
  // Never towards the robot on my hub: go round it and let it swing in behind.
  const Neighbour* behind = towing() ? find(chain_.behind()) : nullptr;
  if (behind && tracked(*behind)) {
    const float bx = std::cos(bearing_of(*behind));
    const float by = std::sin(bearing_of(*behind));
    const float into = dx * bx + dy * by;
    if (into > 0.0f) {
      dx -= into * bx;
      dy -= into * by;
      if (std::hypot(dx, dy) < 0.3f) {  // dead behind: pick a side
        dx = -0.6f * by;
        dy = 0.6f * bx;
      }
    }
  }
  const float length = std::hypot(dx, dy);
  if (length < 0.15f) {
    drive(hal, 0.0f, clampf(p_.seek_turn_gain_per_s * wrap_pi(rest_bearing_rad),
                            -p_.wander_spin_rate_rad_s, p_.wander_spin_rate_rad_s));
    return;
  }
  const float error = std::atan2(dy, dx);
  const float forward = clampf(std::cos(error), 0.0f, 1.0f);
  drive(hal, speed_mm_s * clampf(length, 0.0f, 1.0f) * forward * forward,
        clampf(p_.seek_turn_gain_per_s * error, -p_.wander_spin_rate_rad_s,
               p_.wander_spin_rate_rad_s));
}

const Neighbour* Robot::nearest_ahead() const {
  const Neighbour* best = nullptr;
  for (const Neighbour& n : neighbours_) {
    if (!fresh(n) || std::fabs(n.bearing_rad) > p_.wander_avoid_half_angle_rad) continue;
    if (!best || n.range_mm < best->range_mm) best = &n;
  }
  return best;
}

// A robot on its own reverses, then turns in place. A head with robots behind it only
// turns (it cannot push its chain backwards), far enough that it will not drive into the
// robot behind it. `reverse` forces the reversing: a tail refused my tongue.
void Robot::start_backoff(bool void_left, bool void_right, bool reverse) {
  phase_ = towing() && !reverse ? CATENA_WANDER_TURN : CATENA_WANDER_BACKOFF;
  // With a chain behind me I only pull the tongue clear: there is no edge sensor at the rear.
  backoff_left_mm_ = towing() ? 0.25f * p_.pitch_mm : p_.wander_backoff_distance_mm;
  if (void_left && !void_right) {
    turn_dir_ = -1.0f;  // edge on the left: turn clockwise
  } else if (void_right && !void_left) {
    turn_dir_ = 1.0f;
  } else {
    turn_dir_ = rng_.uniform() < 0.5f ? -1.0f : 1.0f;
  }
  turn_target_rad_ = rng_.uniform(p_.wander_turn_min_rad, p_.wander_turn_max_rad);
  turned_rad_ = 0.0f;
  if (towing()) {
    // Turn on until no robot close by is ahead: the one on my hub, or my own chain curled
    // round me. Avoiding it after the turn would swing me straight back to the edge.
    turn_target_rad_ = p_.wander_turn_min_rad;
    while (turn_target_rad_ < 1.5f * kPi && blocked(turn_dir_ * turn_target_rad_)) {
      turn_target_rad_ += 0.1f;
    }
  }
}

// A robot within avoiding range would be ahead of me after turning by turn_rad.
bool Robot::blocked(float turn_rad) const {
  for (const Neighbour& n : neighbours_) {
    if (!fresh(n) || (n.id != chain_.behind() && range_of(n) > p_.wander_avoid_range_mm)) continue;
    if (std::fabs(wrap_pi(bearing_of(n) - turn_rad)) < p_.wander_avoid_half_angle_rad) return true;
  }
  return false;
}

// Edge avoidance comes first: it overrides cruising and avoiding neighbours. Backing off
// covers a fixed distance and turning happens in place, so neither waits for the edge
// sensors: a robot parallel to an edge would otherwise keep reversing along it.
void Robot::wander(Hal& hal, float dt_s) {
  const bool void_left = !hal.table_left();
  const bool void_right = !hal.table_right();
  float v = 0.0f;
  float w = 0.0f;

  if ((phase_ == CATENA_WANDER_CRUISE || phase_ == CATENA_WANDER_AVOID) &&
      (void_left || void_right)) {
    start_backoff(void_left, void_right, false);
  }

  if (phase_ == CATENA_WANDER_BACKOFF) {
    v = -p_.wander_backoff_speed_mm_s;
    backoff_left_mm_ -= p_.wander_backoff_speed_mm_s * dt_s;
    if (backoff_left_mm_ <= 0.0f) phase_ = CATENA_WANDER_TURN;
  } else if (phase_ == CATENA_WANDER_TURN) {
    w = turn_dir_ * p_.wander_spin_rate_rad_s;
    turned_rad_ += std::fabs(hal.gyro_z_rad_s()) * dt_s;
    if (turned_rad_ >= turn_target_rad_) {
      phase_ = CATENA_WANDER_CRUISE;
      retarget_in_s_ = 0.0f;
    }
  } else {
    // With a chain in tow: slower, and in curves wide enough not to meet its own tail.
    const float cruise = towing() ? lead_speed_mm_s() : p_.wander_speed_mm_s;
    const float circle_mm = static_cast<float>(chain_.size()) * p_.pitch_mm;
    const float turn_limit = towing()
        ? std::fmin(p_.wander_turn_rate_rad_s, 2.0f * kPi * p_.follow_lead_speed_mm_s / circle_mm)
        : p_.wander_turn_rate_rad_s;
    const Neighbour* n = nearest_ahead();
    if (n && n->range_mm < p_.wander_avoid_range_mm) {
      if (phase_ != CATENA_WANDER_AVOID) avoid_dir_ = n->bearing_rad >= 0.0f ? -1.0f : 1.0f;
      phase_ = CATENA_WANDER_AVOID;
      const float span = p_.wander_avoid_range_mm - p_.wander_avoid_stop_range_mm;
      v = cruise * clampf((n->range_mm - p_.wander_avoid_stop_range_mm) / span, 0, 1);
      w = avoid_dir_ * p_.wander_spin_rate_rad_s;
    } else {
      phase_ = CATENA_WANDER_CRUISE;
      retarget_in_s_ -= dt_s;
      if (retarget_in_s_ <= 0.0f) {
        cruise_turn_rad_s_ = rng_.uniform(-p_.wander_turn_rate_rad_s, p_.wander_turn_rate_rad_s);
        retarget_in_s_ = p_.wander_retarget_period_s;
      }
      v = cruise;
      w = clampf(cruise_turn_rad_s_, -turn_limit, turn_limit);
    }
  }
  drive(hal, v, w);
}

void Robot::drive(Hal& hal, float v_mm_s, float w_rad_s) {
  w_rad_s = clampf(w_rad_s, -p_.max_turn_rate_rad_s, p_.max_turn_rate_rad_s);
  float left = v_mm_s - w_rad_s * p_.track_mm * 0.5f;
  float right = v_mm_s + w_rad_s * p_.track_mm * 0.5f;
  const float peak = std::fmax(std::fabs(left), std::fabs(right));
  if (peak > p_.max_speed_mm_s) {
    left *= p_.max_speed_mm_s / peak;
    right *= p_.max_speed_mm_s / peak;
  }
  last_v_mm_s_ = 0.5f * (left + right);
  hal.set_wheels(left, right);
}

}  // namespace catena
