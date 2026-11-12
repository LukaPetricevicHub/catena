// Hal for unit tests: inputs are set by the test, outputs are recorded.
#ifndef CATENA_FAKE_HAL_H
#define CATENA_FAKE_HAL_H

#include <cmath>
#include <vector>

#include "catena/hal.h"
#include "ranging.h"

struct FakeHal final : catena::Hal {
  catena_params_t p;
  uint32_t dt_us = 20000;
  uint32_t entropy_bits = 1;
  float gyro = 0.0f;
  bool left = true, right = true;
  std::vector<catena_rx_t> inbox;
  float wheel_left = 0.0f, wheel_right = 0.0f;
  int flashes = 0;
  uint32_t last_flash_delay_us = 0;
  catena_debug_t debug = {};
  float ring = 0.0f;
  float hall_value = 0.0f;
  std::vector<uint8_t> flash_frame;  // the most recent FLASH payload
  std::vector<catena_tx_t> sent;     // unicast frames of the most recent step (the test clears it)

  FakeHal() { catena_default_params(&p); }

  uint32_t elapsed_us() override { return dt_us; }
  uint32_t entropy() override { return entropy_bits = entropy_bits * 1664525u + 1013904223u; }
  float gyro_z_rad_s() override { return gyro; }
  float ring_angle_rad() override { return ring; }
  float hall() override { return hall_value; }
  bool table_left() override { return left; }
  bool table_right() override { return right; }
  int rx_count() override { return static_cast<int>(inbox.size()); }
  const catena_rx_t& rx(int i) override { return inbox[static_cast<size_t>(i)]; }
  void set_wheels(float l, float r) override {
    wheel_left = l;
    wheel_right = r;
    gyro = (r - l) / p.track_mm;  // ideal wheels: the next step's gyro reads this turn
  }
  void flash(const uint8_t* data, uint8_t len, uint32_t delay_us) override {
    ++flashes;
    last_flash_delay_us = delay_us;
    flash_frame.assign(data, data + len);
  }
  bool send(uint8_t dst, const uint8_t* data, uint8_t len) override {
    catena_tx_t tx = {};
    tx.dst = dst;
    tx.len = len;
    for (uint8_t i = 0; i < len; ++i) tx.data[i] = data[i];
    sent.push_back(tx);
    return true;
  }
  // Frames of one type sent to one robot since `sent` was last cleared.
  int sent_to(uint8_t dst, uint8_t type) const {
    int n = 0;
    for (const catena_tx_t& tx : sent) n += tx.dst == dst && tx.len > 0 && tx.data[0] == type;
    return n;
  }
  void set_debug(const catena_debug_t& d) override { debug = d; }

  float forward() const { return 0.5f * (wheel_left + wheel_right); }
  float turn() const { return (wheel_right - wheel_left) / p.track_mm; }
};

// The receiver model of the simulator, without noise: cosine lobes, inverse square.
inline catena_rx_t flash_from(uint8_t src, float bearing_rad, float range_mm, float ref_mm) {
  catena_rx_t rx = {};
  rx.src = src;
  rx.len = 2;
  rx.has_ir = 1;
  rx.data[0] = CATENA_FRAME_FLASH;
  rx.data[1] = src;
  const float scale = (ref_mm / range_mm) * (ref_mm / range_mm);
  for (int k = 0; k < CATENA_IR_COUNT; ++k) {
    const float c = std::cos(bearing_rad - catena::receiver_angle_rad(k));
    rx.ir[k] = c > 0.0f ? scale * c : 0.0f;
  }
  return rx;
}

// A FLASH with the sender's chain status, as M2 robots announce it.
inline catena_rx_t status_flash(uint8_t src, float bearing_rad, float range_mm, float ref_mm,
                                uint8_t chain_id, uint8_t size, uint8_t index, uint8_t flags,
                                uint8_t target = CATENA_NONE, uint8_t ahead = CATENA_NONE,
                                uint8_t behind = CATENA_NONE) {
  catena_rx_t rx = flash_from(src, bearing_rad, range_mm, ref_mm);
  const uint8_t body[9] = {CATENA_FRAME_FLASH, src, chain_id, size, index, flags, target, ahead,
                           behind};
  rx.len = sizeof body;
  for (int i = 0; i < 9; ++i) rx.data[i] = body[i];
  return rx;
}

inline catena_rx_t unicast(uint8_t src, std::initializer_list<uint8_t> body) {
  catena_rx_t rx = {};
  rx.src = src;
  for (uint8_t b : body) rx.data[rx.len++] = b;
  return rx;
}

#endif
