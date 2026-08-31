#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "nav2_costmap_2d/costmap_layer.hpp"
#include "rclcpp/subscription.hpp"
#include "salus_interfaces/msg/projected_keepout_state.hpp"
#include "salus_navigation_costmap/vector_keepout_geometry.hpp"

namespace salus_navigation_costmap
{
class VectorKeepoutLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  void onInitialize() override;
  void updateBounds(double, double, double, double *, double *, double *, double *) override;
  void updateCosts(nav2_costmap_2d::Costmap2D &, int, int, int, int) override;
  void reset() override;
  bool isClearable() override { return true; }

private:
  void state_callback(const salus_interfaces::msg::ProjectedKeepoutState::SharedPtr message);
  std::vector<Polygon> transform_polygons(const std::vector<Polygon> & source, const std::string & target_frame) const;
  void add_bounds(const std::vector<Polygon> &, const Bounds &, double *, double *, double *, double *) const;
  std::mutex mutex_;
  std::vector<Polygon> polygons_map_, previous_map_, previous_rendered_, cycle_polygons_;
  CostProfile profile_; std::string source_topic_{"/zones_manager/projected_keepouts"};
  std::string map_frame_{"map"};
  rclcpp::Subscription<salus_interfaces::msg::ProjectedKeepoutState>::SharedPtr subscription_;
};
}  // namespace salus_navigation_costmap
