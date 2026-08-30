#include "salus_navigation_costmap/vector_keepout_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace salus_navigation_costmap
{
bool Bounds::intersects(const Bounds & o) const { return min_x <= o.max_x && max_x >= o.min_x && min_y <= o.max_y && max_y >= o.min_y; }
Bounds Bounds::expanded(double m) const { return {min_x - m, min_y - m, max_x + m, max_y + m}; }
Bounds bounds_of(const std::vector<Point> & ring) {
  Bounds b{std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()};
  for (const auto & p : ring) { b.min_x = std::min(b.min_x, p.x); b.min_y = std::min(b.min_y, p.y); b.max_x = std::max(b.max_x, p.x); b.max_y = std::max(b.max_y, p.y); }
  return b;
}
static bool ring_contains(const std::vector<Point> & ring, Point p) {
  bool inside = false;
  for (size_t i = 0, j = ring.size() - 1; i < ring.size(); j = i++) {
    const auto & a = ring[i]; const auto & b = ring[j];
    if (((a.y > p.y) != (b.y > p.y)) && p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x) inside = !inside;
  }
  return inside;
}
bool contains(const Polygon & p, Point point) {
  if (!p.bounds.intersects({point.x, point.y, point.x, point.y}) || !ring_contains(p.outer, point)) return false;
  for (const auto & hole : p.holes) if (ring_contains(hole, point)) return false;
  return true;
}
Polygon transformed(Polygon p, double tx, double ty, double yaw) { const double c = std::cos(yaw), s = std::sin(yaw); auto apply = [&](Point & v) { v = {tx + c * v.x - s * v.y, ty + s * v.x + c * v.y}; }; for (auto & v : p.outer) apply(v); for (auto & ring : p.holes) for (auto & v : ring) apply(v); p.bounds = bounds_of(p.outer); return p; }
std::vector<Polygon> intersecting(const std::vector<Polygon> & polygons, const Bounds & window, double margin) { std::vector<Polygon> result; for (const auto & p : polygons) if (p.bounds.expanded(margin).intersects(window)) result.push_back(p); return result; }
bool add_dirty_bounds(const std::vector<Polygon> & polygons, const Bounds & window, double margin, Bounds * dirty) { bool any = false; for (const auto & p : polygons) { const auto b = p.bounds.expanded(margin); if (!b.intersects(window)) continue; if (!any) { *dirty = b; any = true; } else { dirty->min_x = std::min(dirty->min_x, b.min_x); dirty->min_y = std::min(dirty->min_y, b.min_y); dirty->max_x = std::max(dirty->max_x, b.max_x); dirty->max_y = std::max(dirty->max_y, b.max_y); } } return any; }
unsigned char max_keepout_cost(unsigned char existing, unsigned char keepout) { return keepout == 0 ? existing : (existing == 255 || keepout > existing ? keepout : existing); }
void replace_state(ProjectedState * state, std::vector<Polygon> next, uint64_t revision) { state->previous = std::move(state->current); state->current = std::move(next); state->revision = revision; }
static double segment_distance(Point p, Point a, Point b) {
  const double dx = b.x - a.x, dy = b.y - a.y, d2 = dx * dx + dy * dy;
  const double t = d2 == 0.0 ? 0.0 : std::clamp(((p.x - a.x) * dx + (p.y - a.y) * dy) / d2, 0.0, 1.0);
  return std::hypot(p.x - (a.x + t * dx), p.y - (a.y + t * dy));
}
static double boundary_distance(const Polygon & p, Point q) {
  double best = std::numeric_limits<double>::infinity();
  auto scan = [&](const std::vector<Point> & ring) { for (size_t i = 0; i < ring.size(); ++i) best = std::min(best, segment_distance(q, ring[i], ring[(i + 1) % ring.size()])); };
  scan(p.outer); for (const auto & hole : p.holes) scan(hole); return best;
}
std::vector<unsigned char> rasterize(const std::vector<Polygon> & polygons, const RasterSpec & spec, const CostProfile & profile) {
  std::vector<unsigned char> output(static_cast<size_t>(spec.width) * spec.height, 0);
  const Bounds interest = spec.window.expanded(profile.halo_radius_m);
  std::vector<const Polygon *> relevant;
  relevant.reserve(polygons.size());
  for (const auto & polygon : polygons) if (polygon.bounds.expanded(profile.halo_radius_m).intersects(interest)) relevant.push_back(&polygon);
  for (unsigned int y = 0; y < spec.height; ++y) for (unsigned int x = 0; x < spec.width; ++x) {
    const Point p{spec.window.min_x + (x + 0.5) * spec.resolution, spec.window.min_y + (y + 0.5) * spec.resolution};
    unsigned char cost = 0;
    for (const auto * polygon : relevant) {
      if (contains(*polygon, p)) { cost = std::max(cost, profile.core_cost); continue; }
      if (profile.halo_radius_m > 0.0) {
        const double d = boundary_distance(*polygon, p);
        if (d <= profile.halo_radius_m) {
          const double decay = 99.0 * std::exp(-std::log(99.0 / std::max(1, int(profile.halo_edge_cost))) * d / profile.halo_radius_m);
          cost = std::max(cost, static_cast<unsigned char>(std::clamp(std::lround(decay), long(profile.halo_min_cost), 99L)));
        }
      }
    }
    output[static_cast<size_t>(y) * spec.width + x] = cost;
  }
  return output;
}
}  // namespace salus_navigation_costmap
