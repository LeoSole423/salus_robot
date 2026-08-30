#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace salus_navigation_costmap
{
struct Point { double x; double y; };
struct Bounds {
  double min_x; double min_y; double max_x; double max_y;
  bool intersects(const Bounds & other) const;
  Bounds expanded(double margin) const;
};
struct Polygon { std::string id; std::vector<Point> outer; std::vector<std::vector<Point>> holes; Bounds bounds; };
struct RasterSpec { Bounds window; double resolution; unsigned int width; unsigned int height; };
struct CostProfile { double halo_radius_m{1.5}; unsigned char halo_edge_cost{12}; unsigned char halo_min_cost{1}; unsigned char core_cost{254}; };

Bounds bounds_of(const std::vector<Point> & ring);
bool contains(const Polygon & polygon, Point point);
Polygon transformed(Polygon polygon, double tx, double ty, double yaw);
std::vector<Polygon> intersecting(const std::vector<Polygon> & polygons, const Bounds & window, double margin);
bool add_dirty_bounds(const std::vector<Polygon> & polygons, const Bounds & window, double margin, Bounds * dirty);
unsigned char max_keepout_cost(unsigned char existing, unsigned char keepout_cost);
struct ProjectedState { std::vector<Polygon> current; std::vector<Polygon> previous; uint64_t revision{0}; };
void replace_state(ProjectedState * state, std::vector<Polygon> next, uint64_t revision);
std::vector<unsigned char> rasterize(const std::vector<Polygon> & polygons, const RasterSpec & spec, const CostProfile & profile);
}  // namespace salus_navigation_costmap
