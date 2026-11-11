#include "robot.h"

#include <cmath>

#include "ranging.h"

namespace catena {

namespace {
float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }

float wrap_pi(float a) {
  const float two_pi = 6.28318530718f;
  a = std::fmod(a + 3.14159265359f, two_pi);
  if (a < 0.0f) a += two_pi;
  return a - 3.14159265359f;
}
}  // namespace

Robot::Robot(uint8_t id, const catena_params_t& params)
    : id_(id), p_(params), phase_(CATENA_WANDER_CRUISE) {
  rng_.seed(0x51ED270Bu ^ (static_cast<uint32_t>(id) * 0x01000193u));
}

bool Robot::fresh(const Neighbour& n) const {
  return n.used && now_us_ - n.seen_us <= static_cast<uint64_t>(p_.wander_memory_s * 1e6f);
}

void Robot::step(Hal& hal) {
  const uint32_t dt_us = hal.elapsed_us();
  rng_.mix(hal.entropy());
  if (!started_) {
    started_ = true;
    next_flash_us_ = static_cast<uint64_t>(rng_.uniform() * p_.flash_period_s * 1e6f);
  }
  now_us_ += dt_us;
  const float dt_s = static_cast<float>(dt_us) * 1e-6f;

  // Turning moves every remembered bearing the other way.
  const float turned = hal.gyro_z_rad_s() * dt_s;
  for (Neighbour& n : neighbours_) {
    if (n.used) n.bearing_rad = wrap_pi(n.bearing_rad - turned);
  }

  receive(hal);
  announce(hal);
  wander(hal, dt_s);

  catena_debug_t debug = {};
  debug.state = CATENA_STATE_WANDER;
  debug.detail = phase_;
  for (const Neighbour& n : neighbours_) {
    if (fresh(n)) ++debug.neighbours;
  }
  hal.set_debug(debug);
}

void Robot::receive(Hal& hal) {
  for (int i = 0; i < hal.rx_count(); ++i) {
    const catena_rx_t& rx = hal.rx(i);
    if (rx.len < 2 || rx.data[0] != CATENA_FRAME_FLASH || !rx.has_ir || rx.src == id_) continue;
    FlashFix fix;
    if (!estimate_flash(rx.ir, p_.ir_reference_range_mm, &fix)) continue;
    Neighbour* slot = nullptr;
    Neighbour* oldest = &neighbours_[0];
    for (Neighbour& n : neighbours_) {
      if (n.used && n.id == rx.src) {
        slot = &n;
        break;
      }
      if (!slot && !n.used) slot = &n;
      if (n.seen_us < oldest->seen_us) oldest = &n;
    }
    if (!slot) slot = oldest;
    *slot = Neighbour{true, rx.src, fix.bearing_rad, fix.range_mm, now_us_};
  }
}

void Robot::announce(Hal& hal) {
  const uint64_t tick_us = static_cast<uint64_t>(1e6f / p_.tick_hz);
  if (next_flash_us_ >= now_us_ + tick_us) return;
  const uint32_t delay_us =
      next_flash_us_ > now_us_ ? static_cast<uint32_t>(next_flash_us_ - now_us_) : 0u;
  const uint8_t frame[2] = {CATENA_FRAME_FLASH, id_};
  hal.flash(frame, sizeof frame, delay_us);
  const float period_us =
      p_.flash_period_s * 1e6f * (1.0f + p_.flash_jitter_frac * rng_.uniform(-1.0f, 1.0f));
  next_flash_us_ += static_cast<uint64_t>(period_us);
  if (next_flash_us_ < now_us_) next_flash_us_ = now_us_ + static_cast<uint64_t>(period_us);
}

const Neighbour* Robot::nearest_ahead() const {
  const Neighbour* best = nullptr;
  for (const Neighbour& n : neighbours_) {
    if (!fresh(n) || std::fabs(n.bearing_rad) > p_.wander_avoid_half_angle_rad) continue;
    if (!best || n.range_mm < best->range_mm) best = &n;
  }
  return best;
}

void Robot::start_backoff(bool void_left, bool void_right) {
  phase_ = CATENA_WANDER_BACKOFF;
  backoff_left_mm_ = p_.wander_backoff_distance_mm;
  if (void_left && !void_right) {
    turn_dir_ = -1.0f;  // edge on the left: turn clockwise
  } else if (void_right && !void_left) {
    turn_dir_ = 1.0f;
  } else {
    turn_dir_ = rng_.uniform() < 0.5f ? -1.0f : 1.0f;
  }
  turn_target_rad_ = rng_.uniform(p_.wander_turn_min_rad, p_.wander_turn_max_rad);
  turned_rad_ = 0.0f;
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
    start_backoff(void_left, void_right);
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
    const Neighbour* n = nearest_ahead();
    if (n && n->range_mm < p_.wander_avoid_range_mm) {
      if (phase_ != CATENA_WANDER_AVOID) avoid_dir_ = n->bearing_rad >= 0.0f ? -1.0f : 1.0f;
      phase_ = CATENA_WANDER_AVOID;
      const float span = p_.wander_avoid_range_mm - p_.wander_avoid_stop_range_mm;
      v = p_.wander_speed_mm_s * clampf((n->range_mm - p_.wander_avoid_stop_range_mm) / span, 0, 1);
      w = avoid_dir_ * p_.wander_spin_rate_rad_s;
    } else {
      phase_ = CATENA_WANDER_CRUISE;
      retarget_in_s_ -= dt_s;
      if (retarget_in_s_ <= 0.0f) {
        cruise_turn_rad_s_ = rng_.uniform(-p_.wander_turn_rate_rad_s, p_.wander_turn_rate_rad_s);
        retarget_in_s_ = p_.wander_retarget_period_s;
      }
      v = p_.wander_speed_mm_s;
      w = cruise_turn_rad_s_;
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
  hal.set_wheels(left, right);
}

}  // namespace catena
