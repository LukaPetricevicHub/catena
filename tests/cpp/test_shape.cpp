// A stroke becomes one vertex per robot and a turning angle at each vertex.
#include <cmath>

#include "doctest.h"
#include "shape.h"

using catena::Polygon;
using catena::Stroke;
using catena::make_polygon;

namespace {

const float kPi = 3.14159265f;
const float kLimit = 100.0f * kPi / 180.0f;

Stroke line() {
  Stroke s{};
  for (int k = 0; k < catena::kStrokePoints; ++k) {
    s.xy[2 * k] = static_cast<uint8_t>(10 + 7 * k);
    s.xy[2 * k + 1] = 128;
  }
  return s;
}

Stroke circle() {
  Stroke s{};
  s.closed = true;
  for (int k = 0; k < catena::kStrokePoints; ++k) {
    const float a = 2.0f * kPi * static_cast<float>(k) / catena::kStrokePoints;
    s.xy[2 * k] = static_cast<uint8_t>(std::lround(128.0f + 100.0f * std::cos(a)));
    s.xy[2 * k + 1] = static_cast<uint8_t>(std::lround(128.0f + 100.0f * std::sin(a)));
  }
  return s;
}

float edge(const Polygon& p, int a, int b) {
  return std::hypot(p.x_mm[b] - p.x_mm[a], p.y_mm[b] - p.y_mm[a]);
}

}  // namespace

TEST_CASE("a straight stroke gives twelve vertices one pitch apart and no turning") {
  Polygon p;
  REQUIRE(make_polygon(line(), 12, 80.0f, kLimit, &p));
  CHECK(p.n == 12);
  CHECK_FALSE(p.closed);
  for (int i = 1; i < 12; ++i) CHECK(edge(p, i - 1, i) == doctest::Approx(80.0f).epsilon(0.01));
  for (int i = 0; i < 12; ++i) CHECK(std::fabs(p.turn_rad[i]) < 1e-3f);
  CHECK(p.x_mm[0] + p.x_mm[11] == doctest::Approx(0.0f).epsilon(0.01));  // centred
  CHECK(p.edge_rad[5] == doctest::Approx(0.0f));
}

TEST_CASE("a circle gives every robot the same turn, 360 degrees in all, counter-clockwise") {
  Polygon p;
  REQUIRE(make_polygon(circle(), 12, 80.0f, kLimit, &p));
  CHECK(p.closed);
  float sum = 0.0f, mean_edge = 0.0f;
  for (int i = 0; i < 12; ++i) {
    CHECK(p.turn_rad[i] == doctest::Approx(kPi / 6.0f).epsilon(0.08));
    sum += p.turn_rad[i];
    mean_edge += edge(p, i, (i + 1) % 12) / 12.0f;
  }
  CHECK(sum == doctest::Approx(2.0f * kPi).epsilon(0.001));
  CHECK(mean_edge == doctest::Approx(80.0f).epsilon(0.001));
}

TEST_CASE("the shape follows the number of robots in the chain") {
  Polygon p;
  REQUIRE(make_polygon(circle(), 6, 80.0f, kLimit, &p));
  for (int i = 0; i < 6; ++i) CHECK(p.turn_rad[i] == doctest::Approx(kPi / 3.0f).epsilon(0.08));
  CHECK_FALSE(make_polygon(circle(), 1, 80.0f, kLimit, &p));
  CHECK_FALSE(make_polygon(circle(), catena::kMaxVertices + 1, 80.0f, kLimit, &p));
}

TEST_CASE("turning angles are clamped to the joint limit") {
  Stroke s{};  // out and straight back: a 180 degree turn in the middle
  for (int k = 0; k < catena::kStrokePoints; ++k) {
    s.xy[2 * k] = static_cast<uint8_t>(k < 16 ? 10 + 10 * k : 10 + 10 * (31 - k));
    s.xy[2 * k + 1] = static_cast<uint8_t>(k < 16 ? 100 : 101);
  }
  Polygon p;
  REQUIRE(make_polygon(s, 5, 80.0f, kLimit, &p));
  for (int i = 0; i < 5; ++i) CHECK(std::fabs(p.turn_rad[i]) <= kLimit + 1e-4f);
  CHECK(std::fabs(p.turn_rad[2]) == doctest::Approx(kLimit));
}

TEST_CASE("a stroke without length is no shape") {
  Stroke s{};
  Polygon p;
  CHECK_FALSE(make_polygon(s, 12, 80.0f, kLimit, &p));
}
