#include "chain.h"

namespace catena {

namespace {
uint8_t lower(uint8_t a, uint8_t b) { return a < b ? a : b; }
uint8_t add_sat(uint8_t a, uint8_t b) {
  const int sum = a + b;
  return static_cast<uint8_t>(sum > 254 ? 254 : sum);
}
uint8_t to_byte(float unit) {
  const float x = unit < 0.0f ? 0.0f : (unit > 1.0f ? 1.0f : unit);
  return static_cast<uint8_t>(x * 255.0f + 0.5f);
}
}  // namespace

void ChainLink::set_ahead(uint8_t id, uint64_t now_us) {
  ahead_ = id;
  index_ahead_ = 0;
  min_ahead_ = id;
  hall_ahead_ = 1.0f;
  speed_ahead_mm_s_ = 0.0f;
  ahead_heard_us_ = now_us;
}

void ChainLink::set_behind(uint8_t id, uint8_t count, uint8_t min_id, uint64_t now_us) {
  behind_ = id;
  count_behind_ = count ? count : 1;
  min_behind_ = lower(id, min_id);
  slack_behind_ = 255;
  hold_behind_ = false;
  sight_behind_ = false;
  behind_heard_us_ = now_us;
}

bool ChainLink::on_frame(uint8_t src, const uint8_t* data, uint8_t len, uint64_t now_us) {
  if (len < 5) return false;
  if (data[0] == CATENA_FRAME_LINK_ACK && src == ahead_ && src != CATENA_NONE) {
    index_ahead_ = data[1];
    min_ahead_ = data[2];
    hall_ahead_ = static_cast<float>(data[3]) / 255.0f;
    speed_ahead_mm_s_ = static_cast<float>(static_cast<int8_t>(data[4]));
    ahead_heard_us_ = now_us;
    return true;
  }
  if (data[0] == CATENA_FRAME_LINK_UP && src == behind_ && src != CATENA_NONE) {
    count_behind_ = data[1];
    min_behind_ = data[2];
    hold_behind_ = (data[3] & 1) != 0;
    sight_behind_ = (data[3] & 2) != 0;
    slack_behind_ = data[4];
    behind_heard_us_ = now_us;
    return true;
  }
  return false;
}

uint8_t ChainLink::make_ack(uint8_t* out, float hall, float speed_mm_s) const {
  const float s = speed_mm_s < -127.0f ? -127.0f : (speed_mm_s > 127.0f ? 127.0f : speed_mm_s);
  out[0] = CATENA_FRAME_LINK_ACK;
  out[1] = index();
  out[2] = min_to_here();
  out[3] = to_byte(hall);
  out[4] = static_cast<uint8_t>(static_cast<int8_t>(s));
  return 5;
}

uint8_t ChainLink::make_up(uint8_t* out, bool hold_here, float hall, bool sight_here) const {
  out[0] = CATENA_FRAME_LINK_UP;
  out[1] = count_from_here();
  out[2] = min_from_here();
  out[3] = static_cast<uint8_t>((hold_here || hold_behind() ? 1 : 0) |
                                (sight_here || sight_behind() ? 2 : 0));
  out[4] = behind_ == CATENA_NONE ? 255 : lower(to_byte(hall), slack_behind_);
  return 5;
}

uint8_t ChainLink::index() const { return ahead_ == CATENA_NONE ? 0 : add_sat(index_ahead_, 1); }

uint8_t ChainLink::count_from_here() const {
  return behind_ == CATENA_NONE ? 1 : add_sat(count_behind_, 1);
}

uint8_t ChainLink::min_to_here() const {
  return ahead_ == CATENA_NONE ? id_ : lower(id_, min_ahead_);
}

uint8_t ChainLink::min_from_here() const {
  return behind_ == CATENA_NONE ? id_ : lower(id_, min_behind_);
}

uint8_t ChainLink::size() const { return add_sat(index(), count_from_here()); }

uint8_t ChainLink::chain_id() const { return lower(min_to_here(), min_from_here()); }

float ChainLink::slack(float hall) const {
  if (behind_ == CATENA_NONE) return 1.0f;
  const float behind = static_cast<float>(slack_behind_) / 255.0f;
  return hall < behind ? hall : behind;
}

}  // namespace catena
