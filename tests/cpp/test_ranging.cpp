#include <cmath>

#include "doctest.h"
#include "fake_hal.h"
#include "ranging.h"

TEST_CASE("flash ranging recovers bearing and range from a clean sample") {
  catena_params_t p;
  catena_default_params(&p);
  for (int b = -180; b < 180; b += 7) {
    for (float range : {60.0f, 80.0f, 130.0f, 220.0f}) {
      const float bearing = static_cast<float>(b) * 3.14159265f / 180.0f;
      const catena_rx_t rx = flash_from(3, bearing, range, p.ir_reference_range_mm);
      catena::FlashFix fix;
      REQUIRE(catena::estimate_flash(rx.ir, p.ir_reference_range_mm, &fix));
      const float err = std::remainder(fix.bearing_rad - bearing, 6.2831853f);
      CHECK(std::fabs(err) < 1e-4f);
      CHECK(fix.range_mm == doctest::Approx(range).epsilon(1e-4));
    }
  }
}

TEST_CASE("an IR sample without signal gives no fix") {
  const float dark[CATENA_IR_COUNT] = {};
  catena::FlashFix fix;
  CHECK_FALSE(catena::estimate_flash(dark, 100.0f, &fix));
}

TEST_CASE("the C API exposes the same estimator") {
  catena_params_t p;
  catena_default_params(&p);
  const catena_rx_t rx = flash_from(1, 1.0f, 150.0f, p.ir_reference_range_mm);
  float bearing = 0.0f, range = 0.0f;
  REQUIRE(catena_flash_estimate(&p, rx.ir, &bearing, &range) == 1);
  CHECK(bearing == doctest::Approx(1.0f).epsilon(1e-4));
  CHECK(range == doctest::Approx(150.0f).epsilon(1e-4));
}

// -- objects by reflection (spec section 5) ---------------------------------------------------

namespace {
// What the simulator's reflection model gives for one reflector, without noise.
void add_reflector(float out[CATENA_IR_COUNT], float bearing_rad, float gap_mm, float range_mm) {
  const float strength = (range_mm / gap_mm) * (range_mm / gap_mm);
  for (int k = 0; k < CATENA_IR_COUNT; ++k) {
    const float c = std::cos(bearing_rad - catena::receiver_angle_rad(k));
    if (c > 0.0f) out[k] += strength * c;
  }
}
const bool kAll[CATENA_IR_COUNT] = {true, true, true, true, true, true};
}  // namespace

TEST_CASE("a ball is found at its bearing and gap from the reflection of my own pulse") {
  for (float bearing_deg = -170.0f; bearing_deg <= 180.0f; bearing_deg += 17.0f) {
    for (float gap = 30.0f; gap <= 105.0f; gap += 25.0f) {
      float ir[CATENA_IR_COUNT] = {};
      add_reflector(ir, bearing_deg * 3.14159265f / 180.0f, gap, 110.0f);
      catena::ObjectFix fix;
      REQUIRE(catena::estimate_object(ir, kAll, 110.0f, &fix));
      const float err = std::remainder(fix.bearing_rad * 180.0f / 3.14159265f - bearing_deg, 360.0f);
      CHECK(std::fabs(err) < 1.0f);
      CHECK(fix.gap_mm == doctest::Approx(gap).epsilon(0.02));
    }
  }
}

TEST_CASE("a ball beyond the object range, or no reflection at all, is no object") {
  float ir[CATENA_IR_COUNT] = {};
  catena::ObjectFix fix;
  CHECK_FALSE(catena::estimate_object(ir, kAll, 110.0f, &fix));
  add_reflector(ir, 0.3f, 140.0f, 110.0f);
  CHECK_FALSE(catena::estimate_object(ir, kAll, 110.0f, &fix));
}

TEST_CASE("a neighbour's reflection is left out and the ball in front is still found") {
  float ir[CATENA_IR_COUNT] = {};
  add_reflector(ir, 3.14159265f, 8.0f, 110.0f);  // the robot on my hub, right behind me
  add_reflector(ir, 0.35f, 80.0f, 110.0f);       // a ball ahead, a little to the left
  bool usable[CATENA_IR_COUNT] = {true, true, false, false, false, true};  // rear three masked
  catena::ObjectFix fix;
  REQUIRE(catena::estimate_object(ir, usable, 110.0f, &fix));
  CHECK(fix.bearing_rad == doctest::Approx(0.35f).epsilon(0.05));
  CHECK(fix.gap_mm == doctest::Approx(80.0f).epsilon(0.03));
  // Unmasked, the neighbour wins: that is why the mask is needed.
  REQUIRE(catena::estimate_object(ir, kAll, 110.0f, &fix));
  CHECK(std::fabs(fix.bearing_rad) > 2.5f);
}
