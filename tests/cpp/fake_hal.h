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

  FakeHal() { catena_default_params(&p); }

  uint32_t elapsed_us() override { return dt_us; }
  uint32_t entropy() override { return entropy_bits = entropy_bits * 1664525u + 1013904223u; }
  float gyro_z_rad_s() override { return gyro; }
  float ring_angle_rad() override { return 0.0f; }
  float hall() override { return 0.0f; }
  bool table_left() override { return left; }
  bool table_right() override { return right; }
  int rx_count() override { return static_cast<int>(inbox.size()); }
  const catena_rx_t& rx(int i) override { return inbox[static_cast<size_t>(i)]; }
  void set_wheels(float l, float r) override {
    wheel_left = l;
    wheel_right = r;
    gyro = (r - l) / p.track_mm;  // ideal wheels: the next step's gyro reads this turn
  }
  void flash(const uint8_t*, uint8_t, uint32_t delay_us) override {
    ++flashes;
    last_flash_delay_us = delay_us;
  }
  bool send(uint8_t, const uint8_t*, uint8_t) override { return true; }
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

#endif
