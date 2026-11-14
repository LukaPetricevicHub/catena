// Shape maths (spec 7.4): a stroke of 32 points becomes a polygon with one vertex per
// robot and a target turning angle at each vertex. No sensing and no driving.
#ifndef CATENA_SHAPE_H
#define CATENA_SHAPE_H

#include <stdint.h>

#include "catena/catena_c.h"

namespace catena {

constexpr int kStrokePoints = CATENA_STROKE_POINTS;
constexpr int kMaxVertices = 16;

struct Stroke {
  uint8_t xy[2 * kStrokePoints];  // x0, y0, x1, y1, ... in pad units, y up
  bool closed;
};

struct Polygon {
  int n;
  bool closed;
  float x_mm[kMaxVertices];     // shape frame: origin at the centroid of the vertices
  float y_mm[kMaxVertices];
  float turn_rad[kMaxVertices]; // target turning angle; 0 at the ends of an open shape
  float edge_rad[kMaxVertices]; // direction of the edge from vertex i - 1 to vertex i
};

// Places n vertices evenly along the stroke (n edges if closed, n - 1 if open), scales so
// the mean edge is pitch_mm and clamps the turning angles to limit_rad. False: no shape
// (too few robots, too many, or a stroke without length).
bool make_polygon(const Stroke& stroke, int n, float pitch_mm, float limit_rad, Polygon* out);

}  // namespace catena

#endif
