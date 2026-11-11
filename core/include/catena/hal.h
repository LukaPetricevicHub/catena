// Hardware abstraction: everything behaviour code may sense or do.
// The simulator reaches it through the C API (StructHal in catena_c.cpp); the firmware
// implements it on the ESP32-S3.
#ifndef CATENA_HAL_H
#define CATENA_HAL_H

#include <stdint.h>

#include "catena/catena_c.h"

namespace catena {

class Hal {
 public:
  virtual uint32_t elapsed_us() = 0;  // since the previous step
  virtual uint32_t entropy() = 0;
  virtual float gyro_z_rad_s() = 0;
  virtual float ring_angle_rad() = 0;
  virtual float hall() = 0;
  virtual bool table_left() = 0;
  virtual bool table_right() = 0;
  virtual int rx_count() = 0;
  virtual const catena_rx_t& rx(int i) = 0;

  virtual void set_wheels(float left_mm_s, float right_mm_s) = 0;
  // Broadcast a FLASH frame delay_us from now, then pulse the IR LEDs.
  virtual void flash(const uint8_t* data, uint8_t len, uint32_t delay_us) = 0;
  virtual bool send(uint8_t dst, const uint8_t* data, uint8_t len) = 0;
  virtual void set_debug(const catena_debug_t& debug) = 0;

 protected:
  ~Hal() = default;
};

}  // namespace catena

#endif
