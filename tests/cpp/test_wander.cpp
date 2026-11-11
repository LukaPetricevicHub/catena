#include "doctest.h"
#include "fake_hal.h"
#include "robot.h"

namespace {
void run(catena::Robot& robot, FakeHal& hal, int steps) {
  for (int i = 0; i < steps; ++i) robot.step(hal);
}
}  // namespace

TEST_CASE("a robot announces itself about ten times a second") {
  FakeHal hal;
  catena::Robot robot(4, hal.p);
  run(robot, hal, 500);  // 10 s at 50 Hz
  const float expected = 10.0f / hal.p.flash_period_s;
  CHECK(hal.flashes > 0.9f * expected);
  CHECK(hal.flashes < 1.1f * expected);
}

TEST_CASE("flash timing stays inside the step that announces it") {
  FakeHal hal;
  catena::Robot robot(2, hal.p);
  for (int i = 0; i < 300; ++i) {
    const int before = hal.flashes;
    robot.step(hal);
    if (hal.flashes != before) CHECK(hal.last_flash_delay_us < hal.dt_us);
  }
}

TEST_CASE("a wandering robot on an open table drives forward") {
  FakeHal hal;
  catena::Robot robot(0, hal.p);
  run(robot, hal, 10);
  CHECK(hal.debug.state == CATENA_STATE_WANDER);
  CHECK(hal.debug.detail == CATENA_WANDER_CRUISE);
  CHECK(hal.forward() == doctest::Approx(hal.p.wander_speed_mm_s));
}

TEST_CASE("losing the table on the left makes the robot back off, then turn right") {
  FakeHal hal;
  catena::Robot robot(0, hal.p);
  run(robot, hal, 5);
  hal.left = false;
  robot.step(hal);
  CHECK(robot.wander_phase() == CATENA_WANDER_BACKOFF);
  CHECK(hal.forward() < 0.0f);
  CHECK(hal.turn() == doctest::Approx(0.0f));
  hal.left = true;
  int steps = 0;
  while (robot.wander_phase() == CATENA_WANDER_BACKOFF && steps++ < 500) robot.step(hal);
  const float backed_mm = static_cast<float>(steps + 1) * hal.p.wander_backoff_speed_mm_s * 0.02f;
  CHECK(backed_mm >= hal.p.wander_backoff_distance_mm);
  REQUIRE(robot.wander_phase() == CATENA_WANDER_TURN);
  robot.step(hal);
  CHECK(hal.turn() < 0.0f);  // clockwise, away from the edge on the left
  CHECK(hal.forward() == doctest::Approx(0.0f));
  steps = 0;
  while (robot.wander_phase() == CATENA_WANDER_TURN && steps++ < 1000) robot.step(hal);
  const float turned = static_cast<float>(steps) * hal.p.wander_spin_rate_rad_s * 0.02f;
  CHECK(turned >= hal.p.wander_turn_min_rad);
  CHECK(turned <= hal.p.wander_turn_max_rad + 0.1f);
  CHECK(robot.wander_phase() != CATENA_WANDER_BACKOFF);
}

TEST_CASE("a robot finishes its turn in place even while a sensor still sees no table") {
  // Regression, wander seed 15: parallel to an edge, the robot used to reverse along it
  // until it fell, because the turn restarted the backoff while a sensor saw no table.
  FakeHal hal;
  catena::Robot robot(0, hal.p);
  run(robot, hal, 5);
  hal.left = false;
  int steps = 0;
  while (robot.wander_phase() != CATENA_WANDER_TURN && steps++ < 500) robot.step(hal);
  const float backed_mm = static_cast<float>(steps) * hal.p.wander_backoff_speed_mm_s * 0.02f;
  CHECK(backed_mm <= hal.p.wander_backoff_distance_mm + 1.0f);
  REQUIRE(robot.wander_phase() == CATENA_WANDER_TURN);
  steps = 0;
  while (robot.wander_phase() == CATENA_WANDER_TURN && steps++ < 1000) {
    robot.step(hal);
    CHECK(hal.forward() == doctest::Approx(0.0f));
  }
  CHECK(robot.wander_phase() == CATENA_WANDER_CRUISE);
  robot.step(hal);  // still over the edge: back off again before driving forward
  CHECK(robot.wander_phase() == CATENA_WANDER_BACKOFF);
}

TEST_CASE("a robot never drives forward while a front sensor sees no table") {
  FakeHal hal;
  catena::Robot robot(7, hal.p);
  uint32_t r = 12345u;
  for (int i = 0; i < 20000; ++i) {
    r = r * 1103515245u + 12345u;
    // Mostly table, sometimes one or both sensors over the edge, in runs of a few steps.
    if (i % 7 == 0) {
      const uint32_t pick = (r >> 16) % 10;
      hal.left = pick != 0 && pick != 2;
      hal.right = pick != 1 && pick != 2;
    }
    robot.step(hal);
    if (!hal.left || !hal.right) {
      CHECK(hal.forward() <= 0.0f);
    }
  }
}

TEST_CASE("a neighbour ahead makes the robot steer away and stop when close") {
  FakeHal hal;
  catena::Robot robot(0, hal.p);
  run(robot, hal, 3);
  const float ref = hal.p.ir_reference_range_mm;
  const float between = 0.5f * (hal.p.wander_avoid_range_mm + hal.p.wander_avoid_stop_range_mm);
  hal.inbox.push_back(flash_from(5, 0.3f, between, ref));  // ahead, to the left
  robot.step(hal);
  hal.inbox.clear();
  CHECK(robot.wander_phase() == CATENA_WANDER_AVOID);
  CHECK(hal.turn() < 0.0f);  // turn right, away from it
  CHECK(hal.forward() > 0.0f);
  CHECK(hal.forward() < hal.p.wander_speed_mm_s);

  hal.inbox.push_back(flash_from(5, 0.2f, hal.p.wander_avoid_stop_range_mm - 10.0f, ref));
  robot.step(hal);
  hal.inbox.clear();
  CHECK(hal.forward() == doctest::Approx(0.0f));
  CHECK(hal.debug.neighbours == 1);
}

TEST_CASE("a neighbour behind does not stop a wandering robot") {
  FakeHal hal;
  catena::Robot robot(0, hal.p);
  run(robot, hal, 3);
  hal.inbox.push_back(flash_from(5, 3.0f, 90.0f, hal.p.ir_reference_range_mm));
  robot.step(hal);
  CHECK(robot.wander_phase() == CATENA_WANDER_CRUISE);
  CHECK(hal.forward() == doctest::Approx(hal.p.wander_speed_mm_s));
}

TEST_CASE("remembered neighbours expire") {
  FakeHal hal;
  catena::Robot robot(0, hal.p);
  hal.inbox.push_back(flash_from(5, 0.0f, 120.0f, hal.p.ir_reference_range_mm));
  robot.step(hal);
  hal.inbox.clear();
  CHECK(hal.debug.neighbours == 1);
  run(robot, hal, static_cast<int>(hal.p.wander_memory_s * 50.0f) + 2);
  CHECK(hal.debug.neighbours == 0);
}
