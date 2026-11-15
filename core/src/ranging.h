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

struct ObjectFix {
  float bearing_rad;
  float gap_mm;  // from my base to the object's surface
  bool paired;   // two receivers agreed; false: one receiver only, bearing good to a lobe
};

// An object from the reflection of my own pulse (spec section 5). usable[k] is false for a
// receiver that faces a known neighbour. The strongest usable receiver and its stronger
// usable neighbour give bearing and strength; a reading of 1.0 means a ball at range_mm.
// False: nothing as strong as min_signal.
bool estimate_object(const float reflect[CATENA_IR_COUNT], const bool usable[CATENA_IR_COUNT],
                     float range_mm, ObjectFix* fix, float min_signal = 1.0f);

// Direction of receiver k in the base frame.
float receiver_angle_rad(int k);

}  // namespace catena

#endif
