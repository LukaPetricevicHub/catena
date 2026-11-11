#include "catena/catena_c.h"

#include <string.h>

#include <new>

#include "catena/hal.h"
#include "ranging.h"
#include "robot.h"

namespace {

// Hal over the C API structs: what the simulator sees.
class StructHal final : public catena::Hal {
 public:
  StructHal(const catena_inputs_t* in, catena_outputs_t* out) : in_(in), out_(out) {}

  uint32_t elapsed_us() override { return in_->dt_us; }
  uint32_t entropy() override { return in_->entropy; }
  float gyro_z_rad_s() override { return in_->gyro_z_rad_s; }
  float ring_angle_rad() override { return in_->ring_angle_rad; }
  float hall() override { return in_->hall; }
  bool table_left() override { return in_->table_left != 0; }
  bool table_right() override { return in_->table_right != 0; }
  int rx_count() override { return in_->rx_count < CATENA_RX_MAX ? in_->rx_count : CATENA_RX_MAX; }
  const catena_rx_t& rx(int i) override { return in_->rx[i]; }

  void set_wheels(float left_mm_s, float right_mm_s) override {
    out_->wheel_left_mm_s = left_mm_s;
    out_->wheel_right_mm_s = right_mm_s;
  }
  void flash(const uint8_t* data, uint8_t len, uint32_t delay_us) override {
    if (len > CATENA_FRAME_MAX) len = CATENA_FRAME_MAX;
    out_->flash = 1;
    out_->flash_len = len;
    out_->flash_delay_us = delay_us;
    memcpy(out_->flash_data, data, len);
  }
  bool send(uint8_t dst, const uint8_t* data, uint8_t len) override {
    if (out_->tx_count >= CATENA_TX_MAX || len > CATENA_FRAME_MAX) return false;
    catena_tx_t& tx = out_->tx[out_->tx_count++];
    tx.dst = dst;
    tx.len = len;
    memcpy(tx.data, data, len);
    return true;
  }
  void set_debug(const catena_debug_t& debug) override { out_->debug = debug; }

 private:
  const catena_inputs_t* in_;
  catena_outputs_t* out_;
};

}  // namespace

struct catena_t {
  catena::Robot robot;
};

extern "C" {

void catena_default_params(catena_params_t* out) {
  const catena_params_t defaults = CATENA_PARAMS_DEFAULT;
  *out = defaults;
}

catena_t* catena_create(uint8_t robot_id, const catena_params_t* params) {
  return new (std::nothrow) catena_t{catena::Robot(robot_id, *params)};
}

void catena_step(catena_t* robot, const catena_inputs_t* in, catena_outputs_t* out) {
  memset(out, 0, sizeof *out);
  StructHal hal(in, out);
  robot->robot.step(hal);
}

void catena_destroy(catena_t* robot) { delete robot; }

int catena_flash_estimate(const catena_params_t* params, const float ir[CATENA_IR_COUNT],
                          float* bearing_rad, float* range_mm) {
  catena::FlashFix fix;
  if (!catena::estimate_flash(ir, params->ir_reference_range_mm, &fix)) return 0;
  *bearing_rad = fix.bearing_rad;
  *range_mm = fix.range_mm;
  return 1;
}

size_t catena_sizeof_params(void) { return sizeof(catena_params_t); }
size_t catena_sizeof_inputs(void) { return sizeof(catena_inputs_t); }
size_t catena_sizeof_outputs(void) { return sizeof(catena_outputs_t); }

}  // extern "C"
