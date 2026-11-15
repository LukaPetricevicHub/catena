// One robot's behaviour (spec 7.1 to 7.3): flash announcements, neighbour tracking, the link
// handshake, and the states WANDER, SEEK, DOCK and FOLLOW. Uses only what the Hal senses
// and what confirmed neighbours send.
#ifndef CATENA_ROBOT_H
#define CATENA_ROBOT_H

#include <stdint.h>

#include "catena/hal.h"
#include "chain.h"
#include "rng.h"
#include "shape.h"

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
  // The phone's commands, injected at this robot (spec 10).
  void set_shape(const uint8_t* xy, bool closed, bool active);
  void set_twist(float vx_mm_s, float vy_mm_s, float wz_rad_s);

  uint8_t wander_phase() const { return phase_; }
  const Neighbour* nearest_ahead() const;

 private:
  void receive(Hal& hal);
  void update_links(Hal& hal, float dt_s);
  void find_seeker();
  void announce(Hal& hal);
  void lead(Hal& hal, float dt_s);
  void follow(Hal& hal);
  void shape(Hal& hal, float dt_s);
  void shape_drive(Hal& hal, float vx_mm_s, float vy_mm_s, bool forward_ok);
  void shape_frame(const catena_rx_t& rx);
  void shape_gossip(Hal& hal);
  bool shape_ready();
  const Neighbour* own_tail() const;
  // The robots on either side of me along the shape, the ring's closing link included.
  uint8_t fore() const;
  uint8_t aft() const;
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
  // Spec 7.8: an end that lost its neighbour while a shape is held waits for the other end.
  bool shape_wanted() const { return shape_version_ != 0 && (shape_flags_ & 2) && chunks_ == 0x0F; }
  bool orphan_tail() const { return chain_.behind() == CATENA_NONE && now_us_ < orphan_tail_until_us_; }
  bool orphan_head() const { return chain_.ahead() == CATENA_NONE && now_us_ < orphan_head_until_us_; }
  void lost_ahead();
  void lost_behind();
  const Neighbour* orphan_goal() const;
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
  bool traced_here_ = false;  // I see a lower-ranked head that is seeking
  bool run_in_ = false;  // towing: on the straight run from the outer circle to the standoff

  // Shape (spec 7.4, 7.5).
  Stroke stroke_ = {};
  uint8_t shape_version_ = 0;     // 0: none yet
  uint8_t shape_flags_ = 0;       // 1 closed, 2 active
  uint8_t chunks_ = 0;            // which quarters of the stroke have arrived
  uint8_t twist_seq_ = 0;
  int8_t twist_[3] = {};          // vx, vy in mm/s, wz in 0.01 rad/s
  Polygon poly_ = {};
  bool poly_ok_ = false;
  uint8_t poly_version_ = 0;
  uint8_t gossip_count_ = 0;
  float psi_rad_ = 0.0f;          // base heading by gyro, from an arbitrary start
  float e_self_ = 0.0f;           // my turning error
  bool has_turn_ = false;
  float e_fore_ = 0.0f;
  float e_aft_ = 0.0f;
  uint64_t e_fore_us_ = 0;
  uint64_t aft_us_ = 0;
  float aft_tongue_rad_ = 0.0f;   // tongue direction of the robot behind, in its own psi
  float aft_offset_rad_ = 0.0f;   // its psi against mine, learnt from flash bearings
  int aft_samples_ = 0;
  uint8_t aft_id_ = CATENA_NONE;
  uint8_t closure_ = CATENA_NONE; // other end of the ring's closing link
  uint64_t closure_heard_us_ = 0;
  float closure_hall_ = 1.0f;
  uint8_t close_req_id_ = CATENA_NONE;
  uint64_t close_req_us_ = 0;
  float push_in_x_ = 0.0f;        // strongest edge push heard lately, shape frame, mm/s
  float push_in_y_ = 0.0f;
  uint64_t push_in_us_ = 0;
  float edge_push_x_ = 0.0f;      // my own push, fixed when the edge was first seen
  float edge_push_y_ = 0.0f;
  uint64_t edge_until_us_ = 0;
  float push_out_x_ = 0.0f;
  float push_out_y_ = 0.0f;
  float ramp_ = 0.0f;             // 0..1, share of the commanded travel speed in use
  uint64_t orphan_tail_until_us_ = 0;
  uint64_t orphan_tail_limit_us_ = 0;
  uint64_t orphan_head_until_us_ = 0;
  uint64_t orphan_head_limit_us_ = 0;
  uint64_t closure_split_us_ = 0;  // since when the ring's other end reports another chain
  float owed_mm_ = 0.0f;          // commanded travel not yet driven, along owed_x/y (shape frame)
  float owed_x_ = 1.0f;
  float owed_y_ = 0.0f;
  uint8_t shape_detail_ = CATENA_SHAPE_HOLD;
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
