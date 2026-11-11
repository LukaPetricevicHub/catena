#include "ranging.h"

#include <cmath>

namespace catena {

namespace {
constexpr float kPi = 3.14159265358979f;

// Length of the vector sum for a sender straight ahead at the reference range, with
// cosine-lobe receivers. For six receivers it is 1.5 at every bearing.
float lobe_gain() {
  float x = 0.0f;
  for (int k = 0; k < CATENA_IR_COUNT; ++k) {
    const float c = std::cos(receiver_angle_rad(k));
    if (c > 0.0f) x += c * c;
  }
  return x;
}
}  // namespace

float receiver_angle_rad(int k) { return 2.0f * kPi * static_cast<float>(k) / CATENA_IR_COUNT; }

bool estimate_flash(const float ir[CATENA_IR_COUNT], float reference_range_mm, FlashFix* fix) {
  float vx = 0.0f;
  float vy = 0.0f;
  for (int k = 0; k < CATENA_IR_COUNT; ++k) {
    const float a = receiver_angle_rad(k);
    vx += ir[k] * std::cos(a);
    vy += ir[k] * std::sin(a);
  }
  const float length = std::sqrt(vx * vx + vy * vy);
  if (!(length > 1e-9f)) return false;
  fix->bearing_rad = std::atan2(vy, vx);
  fix->range_mm = reference_range_mm * std::sqrt(lobe_gain() / length);
  return true;
}

}  // namespace catena
