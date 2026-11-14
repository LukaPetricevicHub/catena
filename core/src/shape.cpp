#include "shape.h"

#include <cmath>

namespace catena {

bool make_polygon(const Stroke& stroke, int n, float pitch_mm, float limit_rad, Polygon* out) {
  if (n < 2 || n > kMaxVertices) return false;
  const int segments = kStrokePoints - 1 + (stroke.closed ? 1 : 0);
  float cumulative[kStrokePoints + 1];
  cumulative[0] = 0.0f;
  auto px = [&](int k) { return static_cast<float>(stroke.xy[2 * (k % kStrokePoints)]); };
  auto py = [&](int k) { return static_cast<float>(stroke.xy[2 * (k % kStrokePoints) + 1]); };
  for (int k = 0; k < segments; ++k) {
    cumulative[k + 1] = cumulative[k] + std::hypot(px(k + 1) - px(k), py(k + 1) - py(k));
  }
  const float total = cumulative[segments];
  if (total < 1.0f) return false;

  const int edges = stroke.closed ? n : n - 1;
  int k = 0;
  for (int i = 0; i < n; ++i) {
    const float s = total * static_cast<float>(i) / static_cast<float>(edges);
    while (k < segments - 1 && cumulative[k + 1] < s) ++k;
    const float span = cumulative[k + 1] - cumulative[k];
    const float t = span > 0.0f ? (s - cumulative[k]) / span : 0.0f;
    out->x_mm[i] = px(k) + t * (px(k + 1) - px(k));
    out->y_mm[i] = py(k) + t * (py(k + 1) - py(k));
  }
  float mean_edge = 0.0f;
  for (int e = 0; e < edges; ++e) {
    const int a = e, b = (e + 1) % n;
    mean_edge += std::hypot(out->x_mm[b] - out->x_mm[a], out->y_mm[b] - out->y_mm[a]);
  }
  mean_edge /= static_cast<float>(edges);
  if (mean_edge <= 0.0f) return false;
  const float scale = pitch_mm / mean_edge;
  float cx = 0.0f, cy = 0.0f;
  for (int i = 0; i < n; ++i) {
    cx += out->x_mm[i];
    cy += out->y_mm[i];
  }
  cx /= static_cast<float>(n);
  cy /= static_cast<float>(n);
  for (int i = 0; i < n; ++i) {
    out->x_mm[i] = (out->x_mm[i] - cx) * scale;
    out->y_mm[i] = (out->y_mm[i] - cy) * scale;
  }
  for (int i = 0; i < n; ++i) {
    const int prev = (i + n - 1) % n, next = (i + 1) % n;
    const float ux = out->x_mm[i] - out->x_mm[prev], uy = out->y_mm[i] - out->y_mm[prev];
    const float wx = out->x_mm[next] - out->x_mm[i], wy = out->y_mm[next] - out->y_mm[i];
    const bool end = !stroke.closed && (i == 0 || i == n - 1);
    float turn = end ? 0.0f : std::atan2(ux * wy - uy * wx, ux * wx + uy * wy);
    if (turn > limit_rad) turn = limit_rad;
    if (turn < -limit_rad) turn = -limit_rad;
    out->turn_rad[i] = turn;
    // The open shape has no edge into vertex 0; give it the edge that leaves it.
    out->edge_rad[i] = !stroke.closed && i == 0 ? std::atan2(wy, wx) : std::atan2(uy, ux);
  }
  out->n = n;
  out->closed = stroke.closed;
  return true;
}

}  // namespace catena
