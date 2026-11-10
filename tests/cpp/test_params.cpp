#include "catena/catena_c.h"
#include "doctest.h"

TEST_CASE("default parameters come from the generated header") {
  catena_params_t p;
  catena_default_params(&p);
  const catena_params_t expected = CATENA_PARAMS_DEFAULT;
  CHECK(p.pitch_mm == expected.pitch_mm);
  CHECK(p.pitch_mm > 0.0f);
  CHECK(catena_sizeof_params() == sizeof(catena_params_t));
}

TEST_CASE("a robot can unlink by driving away") {
  // Decision D1: the tip magnet must hold less than one robot can pull.
  catena_params_t p;
  catena_default_params(&p);
  CHECK(p.hold_force_n > 0.0f);
  CHECK(p.hold_force_n < p.traction_force_n);
}
