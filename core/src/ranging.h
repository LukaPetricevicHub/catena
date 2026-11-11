// Flash ranging (spec section 5): bearing from the weighted vector sum of the six receivers,
// range from the inverse-square law on the length of that sum.
#ifndef CATENA_RANGING_H
#define CATENA_RANGING_H

#include "catena/catena_c.h"

namespace catena {

struct FlashFix {
  float bearing_rad;
  float range_mm;
};

// Returns false if the sample carries no usable signal.
bool estimate_flash(const float ir[CATENA_IR_COUNT], float reference_range_mm, FlashFix* fix);

// Direction of receiver k in the base frame.
float receiver_angle_rad(int k);

}  // namespace catena

#endif
