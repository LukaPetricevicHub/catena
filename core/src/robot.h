// One robot's behaviour (spec 7.1 to 7.3): flash announcements, neighbour tracking, the link
// handshake, and the states WANDER, SEEK, DOCK and FOLLOW. Uses only what the Hal senses
// and what confirmed neighbours send.
#ifndef CATENA_ROBOT_H
#define CATENA_ROBOT_H

#include <stdint.h>

#include "catena/hal.h"
#include "chain.h"
#include "rng.h"

namespace catena {

struct Neighbour {
  bool used;
  uint8_t id;
  float bearing_rad;  // latest flash measurement, base frame, carried along by dead reckoning
  float range_mm;
  float x_mm;         // filtered position in the base frame, carried along the same way
  float y_mm;
  uint64_t seen_us;   // last clean flash measurement
  bool has_info;      // chain status from its FLASH frame
  uint8_t chain_id;
  uint8_t size;
  uint8_t index;
  uint8_t flags;
  uint8_t target;
  uint8_t ahead;
  uint8_t behind;
  uint64_t info_us;
  float guide_rad;    // GUIDE: my offset from the middle of its acceptance zone
  uint64_t guide_us;
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
  void update_links(Hal& hal, float dt_s);
  void find_seeker();
  void announce(Hal& hal);
  void lead(Hal& hal, float dt_s);
  void follow(Hal& hal);
  void wander(Hal& hal, float dt_s);
  void seek(Hal& hal, const Neighbour& goal, float dt_s);
  void dock(Hal& hal, const Neighbour& goal, float range_mm, float bearing_rad, float offset_rad,
            bool wait, float dt_s);
  void start_backoff(bool void_left, bool void_right, bool reverse);
  void steer(Hal& hal, float dx, float dy, float speed_mm_s, float rest_bearing_rad);
  void drive(Hal& hal, float v_mm_s, float w_rad_s);
  const Neighbour* pick_goal() const;
  const Neighbour* pick_adoptee() const;
  bool rival(const Neighbour& goal) const;
  bool tow_clear(float bearing_rad) const;
  bool blocked(float turn_rad) const;
  bool accepts_link(const Neighbour& n) const;
  float lead_speed_mm_s() const;
  Neighbour* find(uint8_t id);
  const Neighbour* find(uint8_t id) const;
  bool fresh(const Neighbour& n) const;      // measured recently enough to steer round it
  bool tracked(const Neighbour& n) const;    // position good enough to drive to
  bool confirmed(const Neighbour& n) const;  // seen flashing: its radio frames are accepted
  bool towing() const { return chain_.behind() != CATENA_NONE; }

  uint8_t id_;
  catena_params_t p_;
  Rng rng_;
  ChainLink chain_;
  bool started_ = false;
  uint64_t now_us_ = 0;
  uint64_t next_flash_us_ = 0;
  Neighbour neighbours_[kMaxNeighbours] = {};
  float ring_rad_ = 0.0f;
  float hall_ = 0.0f;
  float last_v_mm_s_ = 0.0f;

  // Link layer.
  uint8_t req_id_ = CATENA_NONE;  // who asked to link, and what hangs behind it
  uint8_t req_chain_ = 0;
  uint8_t req_size_ = 1;
  uint64_t req_us_ = 0;
  float tip_on_s_ = 0.0f;         // how long the hall sensor has reported a tip
  float tip_off_s_ = 0.0f;
  float mislinked_s_ = 0.0f;      // how long the robot ahead has not been where a link puts it
  uint8_t refused_id_ = CATENA_NONE;  // did not take the link I offered
  uint64_t refused_until_us_ = 0;
  bool hold_here_ = false;        // a robot is docking onto my hub
  uint8_t seeker_ = CATENA_NONE;

  // Behaviour.
  uint8_t state_ = CATENA_STATE_WANDER;
  uint8_t phase_;                 // WANDER sub-state
  bool holding_ = false;
  uint8_t goal_ = CATENA_NONE;    // robot being approached
  uint8_t target_ = CATENA_NONE;  // the tail announced as my target
  uint8_t seek_detail_ = CATENA_SEEK_TAIL;
  uint8_t dock_phase_ = CATENA_DOCK_ALIGN;
  uint64_t phase_start_us_ = 0;
  int attempts_ = 0;
  float driven_mm_ = 0.0f;
  float leave_left_mm_ = 0.0f;
  float approach_bias_rad_ = 0.0f;
  bool run_in_ = false;  // towing: on the straight run from the outer circle to the standoff

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
