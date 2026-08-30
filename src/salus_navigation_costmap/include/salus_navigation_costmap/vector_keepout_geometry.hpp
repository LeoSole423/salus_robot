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
std::vector<unsigned char> rasterize(const std::vector<Polygon> & polygons, const RasterSpec & spec, const CostProfile & profile);
}  // namespace salus_navigation_costmap
