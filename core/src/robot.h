// One robot's behaviour. M1: flash announcements, neighbour measurements, WANDER with edge
// avoidance. Uses only what the Hal senses and what neighbours send.
#ifndef CATENA_ROBOT_H
#define CATENA_ROBOT_H

#include <stdint.h>

#include "catena/hal.h"
#include "rng.h"

namespace catena {

struct Neighbour {
  bool used;
  uint8_t id;
  float bearing_rad;  // base frame, kept current by the gyro between flashes
  float range_mm;
  uint64_t seen_us;
};

constexpr int kMaxNeighbours = 16;

class Robot {
 public:
  Robot(uint8_t id, const catena_params_t& params);
  void step(Hal& hal);

  uint8_t wander_phase() const { return phase_; }
  const Neighbour* nearest_ahead() const;

 private:
  void receive(Hal& hal);
  void announce(Hal& hal);
  void wander(Hal& hal, float dt_s);
  void start_backoff(bool void_left, bool void_right);
  void drive(Hal& hal, float v_mm_s, float w_rad_s);
  bool fresh(const Neighbour& n) const;

  uint8_t id_;
  catena_params_t p_;
  Rng rng_;
  bool started_ = false;
  uint64_t now_us_ = 0;
  uint64_t next_flash_us_ = 0;
  Neighbour neighbours_[kMaxNeighbours] = {};

  uint8_t phase_;
  float cruise_turn_rad_s_ = 0.0f;
  float retarget_in_s_ = 0.0f;
  float avoid_dir_ = 1.0f;
  float backoff_left_mm_ = 0.0f;
  float turn_dir_ = 1.0f;
  float turn_target_rad_ = 0.0f;
  float turned_rad_ = 0.0f;
};

}  // namespace catena

#endif
