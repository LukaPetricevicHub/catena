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

bool estimate_object(const float reflect[CATENA_IR_COUNT], const bool usable[CATENA_IR_COUNT],
                     float range_mm, ObjectFix* fix) {
  int best = -1;
  for (int k = 0; k < CATENA_IR_COUNT; ++k) {
    if (usable[k] && (best < 0 || reflect[k] > reflect[best])) best = k;
  }
  if (best < 0 || !(reflect[best] > 0.0f)) return false;
  const int left = (best + 1) % CATENA_IR_COUNT;
  const int right = (best + CATENA_IR_COUNT - 1) % CATENA_IR_COUNT;
  int side = 0;  // which neighbour to use: +1 the next receiver counter-clockwise, -1 the other
  if (usable[left] && (!usable[right] || reflect[left] >= reflect[right])) side = 1;
  else if (usable[right]) side = -1;
  // Cosine lobes one spacing apart: r_best = A cos(d), r_side = A cos(spacing - d).
  const float spacing = 2.0f * kPi / CATENA_IR_COUNT;
  float offset = 0.0f;
  if (side != 0) {
    const float ratio = reflect[side > 0 ? left : right] / reflect[best];
    offset = std::atan2(ratio - std::cos(spacing), std::sin(spacing));
    if (offset < 0.0f) offset = 0.0f;
    if (offset > 0.5f * spacing) offset = 0.5f * spacing;
  }
  const float strength = reflect[best] / std::cos(offset);
  if (strength < 1.0f) return false;
  float bearing = receiver_angle_rad(best) + static_cast<float>(side) * offset;
  if (bearing > kPi) bearing -= 2.0f * kPi;
  fix->bearing_rad = bearing;
  fix->gap_mm = range_mm / std::sqrt(strength);
  return true;
}

}  // namespace catena
