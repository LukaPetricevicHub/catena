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
