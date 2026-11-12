// Link layer of one robot in a chain (spec 7.2, 7.3): who is ahead and who is behind, and
// the index, count and chain id that gossip along the links. No sensing and no driving.
#ifndef CATENA_CHAIN_H
#define CATENA_CHAIN_H

#include <stdint.h>

#include "catena/catena_c.h"

namespace catena {

struct Rank {
  uint8_t size;
  uint8_t chain_id;  // lowest robot id in the chain
};

// Larger size wins, lower chain id breaks ties. A strict order: never true both ways.
inline bool outranks(Rank a, Rank b) {
  return a.size != b.size ? a.size > b.size : a.chain_id < b.chain_id;
}

class ChainLink {
 public:
  explicit ChainLink(uint8_t id) : id_(id) {}

  uint8_t id() const { return id_; }
  uint8_t ahead() const { return ahead_; }    // robot whose hub my tongue sits on
  uint8_t behind() const { return behind_; }  // robot whose tongue sits on my hub
  void set_ahead(uint8_t id, uint64_t now_us);
  // `count` and `min_id`: what is known of the chain that hangs behind, until it reports.
  void set_behind(uint8_t id, uint8_t count, uint8_t min_id, uint64_t now_us);
  void clear_ahead() { ahead_ = CATENA_NONE; }
  void clear_behind() { behind_ = CATENA_NONE; }

  // LINK_ACK from `ahead` or LINK_UP from `behind`. False: not from a linked neighbour.
  bool on_frame(uint8_t src, const uint8_t* data, uint8_t len, uint64_t now_us);
  // Frames for the robot behind and the robot ahead; return the length.
  uint8_t make_ack(uint8_t* out, float hall, float speed_mm_s) const;
  uint8_t make_up(uint8_t* out, bool hold_here, float hall, bool sight_here) const;

  uint8_t index() const;    // the head is 0
  uint8_t size() const;
  uint8_t chain_id() const;
  Rank rank() const { return Rank{size(), chain_id()}; }

  bool hold_behind() const { return behind_ != CATENA_NONE && hold_behind_; }
  // A robot behind me sees a higher-ranked chain.
  bool sight_behind() const { return behind_ != CATENA_NONE && sight_behind_; }
  // Lowest hall reading of all links behind this robot; `hall` is its own hub.
  float slack(float hall) const;
  float ahead_hall() const { return hall_ahead_; }
  float ahead_speed_mm_s() const { return speed_ahead_mm_s_; }
  uint64_t ahead_heard_us() const { return ahead_heard_us_; }
  uint64_t behind_heard_us() const { return behind_heard_us_; }

 private:
  uint8_t count_from_here() const;
  uint8_t min_to_here() const;
  uint8_t min_from_here() const;

  uint8_t id_;
  uint8_t ahead_ = CATENA_NONE;
  uint8_t behind_ = CATENA_NONE;
  uint8_t index_ahead_ = 0;
  uint8_t min_ahead_ = CATENA_NONE;
  uint8_t count_behind_ = 0;
  uint8_t min_behind_ = CATENA_NONE;
  uint8_t slack_behind_ = 255;
  bool hold_behind_ = false;
  bool sight_behind_ = false;
  float hall_ahead_ = 1.0f;
  float speed_ahead_mm_s_ = 0.0f;
  uint64_t ahead_heard_us_ = 0;
  uint64_t behind_heard_us_ = 0;
};

}  // namespace catena

#endif
