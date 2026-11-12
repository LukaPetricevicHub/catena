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
  if (chain_.ahead() != CATENA_NONE) {
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
    hal.send(chain_.behind(), frame, chain_.make_ack(frame, hall_, last_v_mm_s_ * along));
  }
  if (chain_.ahead() != CATENA_NONE) {
    // Tell the head when I can see a higher-ranked chain that it may not.
    bool sight = false;
    const Rank mine = chain_.rank();
    for (const Neighbour& n : neighbours_) {
      sight = sight || (tracked(n) && n.has_info && n.chain_id != mine.chain_id &&
                        outranks(Rank{n.size, n.chain_id}, mine));
    }
    hal.send(chain_.ahead(), frame, chain_.make_up(frame, hold_here_, hall_, sight, traced_here_));
  }

  catena_debug_t debug = {};
  debug.state = state_;
  if (state_ == CATENA_STATE_WANDER) {
    debug.detail = holding_ ? CATENA_WANDER_HOLD : phase_;
  } else if (state_ == CATENA_STATE_SEEK) {
    debug.detail = seek_detail_;
  } else if (state_ == CATENA_STATE_DOCK) {
    debug.detail = dock_phase_;
  }
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
    const bool linked = rx.src == chain_.ahead() || rx.src == chain_.behind();
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
    } else if (type == CATENA_FRAME_LINK_ACK) {
      if (chain_.ahead() == CATENA_NONE && accepts_link(*n)) {
        chain_.set_ahead(rx.src, now_us_);
        mislinked_s_ = 0.0f;
      }
      chain_.on_frame(rx.src, rx.data, rx.len, now_us_);
    } else if (type == CATENA_FRAME_LINK_UP) {
      chain_.on_frame(rx.src, rx.data, rx.len, now_us_);
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

  // Behind: the hall sensor says whether a tip is there; frames say whose it is.
  if (chain_.behind() != CATENA_NONE) {
    if (tip_off_s_ * p_.tick_hz > 3.0f) {
      chain_.clear_behind();
    } else if (now_us_ - chain_.behind_heard_us() > timeout_us) {
      refused_id_ = chain_.behind();
      refused_until_us_ = now_us_ + 4 * timeout_us;
      chain_.clear_behind();
      tip_on_s_ = 0.0f;
    }
  } else if (hall_ > p_.hall_link) {
    const Neighbour* asked = req_id_ != CATENA_NONE && now_us_ - req_us_ <= timeout_us
                                 ? find(req_id_) : nullptr;
    if (asked && tracked(*asked) && range_of(*asked) < p_.link_range_mm) {
      chain_.set_behind(req_id_, req_size_, req_chain_, now_us_);
      req_id_ = CATENA_NONE;
    } else if (tip_on_s_ >= p_.adopt_delay_s) {
      // A tip landed that nobody announced (robots bump into each other): offer the link.
      const Neighbour* n = pick_adoptee();
      if (n && chain_.size() > 1 && n->chain_id == chain_.chain_id()) {
        const uint8_t nak[1] = {CATENA_FRAME_LINK_NAK};  // my own head: no rings before M3
        hal.send(n->id, nak, sizeof nak);
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
    if (lost) chain_.clear_ahead();
  }
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
  if (chain_.behind() != CATENA_NONE || hall_ > p_.hall_link) return;
  const Rank mine = chain_.rank();
  float best_mm = p_.seek_hold_range_mm;
  for (const Neighbour& n : neighbours_) {
    if (!tracked(n) || !n.has_info || n.target != id_ || !(n.flags & CATENA_FLAG_HEAD)) continue;
    if (now_us_ - n.info_us > to_us(p_.wander_memory_s)) continue;
    if (!outranks(mine, Rank{n.size, n.chain_id})) continue;
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
  if (chain_.ahead() == CATENA_NONE) flags |= CATENA_FLAG_HEAD;
  if (chain_.behind() == CATENA_NONE && hall_ <= p_.hall_link) flags |= CATENA_FLAG_TAIL;
  if (state_ == CATENA_STATE_DOCK) flags |= CATENA_FLAG_DOCKING;
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
