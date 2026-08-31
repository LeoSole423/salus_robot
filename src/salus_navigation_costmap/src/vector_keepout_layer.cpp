#include "salus_navigation_costmap/vector_keepout_layer.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "pluginlib/class_list_macros.hpp"
#include "tf2/exceptions.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace salus_navigation_costmap
{
void VectorKeepoutLayer::onInitialize() {
  auto node = node_.lock(); if (!node) throw std::runtime_error("costmap lifecycle node expired");
  declareParameter("enabled", rclcpp::ParameterValue(false)); declareParameter("source_topic", rclcpp::ParameterValue(source_topic_));
  declareParameter("map_frame", rclcpp::ParameterValue(map_frame_)); declareParameter("halo_radius_m", rclcpp::ParameterValue(1.5));
  declareParameter("halo_edge_cost", rclcpp::ParameterValue(12)); declareParameter("halo_min_cost", rclcpp::ParameterValue(1));
  node->get_parameter(name_ + ".enabled", enabled_); node->get_parameter(name_ + ".source_topic", source_topic_); node->get_parameter(name_ + ".map_frame", map_frame_);
  node->get_parameter(name_ + ".halo_radius_m", profile_.halo_radius_m); int edge, minimum; node->get_parameter(name_ + ".halo_edge_cost", edge); node->get_parameter(name_ + ".halo_min_cost", minimum);
  profile_.halo_radius_m = std::max(0.0, profile_.halo_radius_m); profile_.halo_edge_cost = static_cast<unsigned char>(std::clamp(edge, 1, 99)); profile_.halo_min_cost = static_cast<unsigned char>(std::clamp(minimum, 1, int(profile_.halo_edge_cost)));
  current_ = true;
  rclcpp::QoS qos(1); qos.reliable().transient_local();
  subscription_ = node->create_subscription<salus_interfaces::msg::ProjectedKeepoutState>(source_topic_, qos, std::bind(&VectorKeepoutLayer::state_callback, this, std::placeholders::_1));
}
void VectorKeepoutLayer::state_callback(const salus_interfaces::msg::ProjectedKeepoutState::SharedPtr msg) {
  if (msg->header.frame_id != map_frame_) { RCLCPP_ERROR(logger_, "Ignoring projected keepouts in '%s', expected '%s'", msg->header.frame_id.c_str(), map_frame_.c_str()); return; }
  std::vector<Polygon> next; next.reserve(msg->polygons.size());
  for (const auto & in : msg->polygons) {
    Polygon p; p.id = in.zone_id; for (const auto & v : in.outer.points) p.outer.push_back({v.x, v.y});
    for (const auto & hole : in.holes) { std::vector<Point> ring; for (const auto & v : hole.points) ring.push_back({v.x, v.y}); if (ring.size() >= 3) p.holes.push_back(std::move(ring)); }
    if (p.outer.size() >= 3) { p.bounds = bounds_of(p.outer); next.push_back(std::move(p)); }
  }
  std::lock_guard<std::mutex> lock(mutex_); ProjectedState state{polygons_map_, previous_map_, 0}; replace_state(&state, std::move(next), msg->revision); polygons_map_ = std::move(state.current); previous_map_ = std::move(state.previous); current_ = true;
}
std::vector<Polygon> VectorKeepoutLayer::transform_polygons(const std::vector<Polygon> & source, const std::string & target) const {
  if (target == map_frame_) return source;
  const auto tf = tf_->lookupTransform(target, map_frame_, tf2::TimePointZero);
  const auto & q = tf.transform.rotation; const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  std::vector<Polygon> result; result.reserve(source.size()); for (const auto & p : source) result.push_back(transformed(p, tf.transform.translation.x, tf.transform.translation.y, yaw));
  return result;
}
void VectorKeepoutLayer::add_bounds(const std::vector<Polygon> & polygons, const Bounds & window, double * min_x, double * min_y, double * max_x, double * max_y) const {
  Bounds dirty{}; if (add_dirty_bounds(polygons, window, profile_.halo_radius_m, &dirty)) { *min_x = std::min(*min_x, dirty.min_x); *min_y = std::min(*min_y, dirty.min_y); *max_x = std::max(*max_x, dirty.max_x); *max_y = std::max(*max_y, dirty.max_y); }
}
void VectorKeepoutLayer::updateBounds(double, double, double, double * min_x, double * min_y, double * max_x, double * max_y) {
  if (!enabled_) return; std::vector<Polygon> current, old, rendered;
  { std::lock_guard<std::mutex> lock(mutex_); current = polygons_map_; old = previous_map_; rendered = previous_rendered_; }
  try { current = transform_polygons(current, layered_costmap_->getGlobalFrameID()); old = transform_polygons(old, layered_costmap_->getGlobalFrameID()); }
  catch (const tf2::TransformException & error) { current_ = false; RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000, "Keepout transform unavailable: %s", error.what()); return; }
  const auto * grid = layered_costmap_->getCostmap(); const Bounds window{grid->getOriginX(), grid->getOriginY(), grid->getOriginX() + grid->getSizeInMetersX(), grid->getOriginY() + grid->getSizeInMetersY()};
  // `rendered` is in the costmap frame from the preceding update. Including it
  // makes a changed map->odom transform dirty its former cells as well as its
  // current cells. The bounded rolling master will clip these bounds.
  add_bounds(current, window, min_x, min_y, max_x, max_y); add_bounds(old, window, min_x, min_y, max_x, max_y); add_bounds(rendered, window, min_x, min_y, max_x, max_y);
  { std::lock_guard<std::mutex> lock(mutex_); previous_map_.clear(); previous_rendered_ = current; cycle_polygons_ = current; }
  current_ = true;
}
void VectorKeepoutLayer::updateCosts(nav2_costmap_2d::Costmap2D & master, int min_i, int min_j, int max_i, int max_j) {
  if (!enabled_ || min_i >= max_i || min_j >= max_j || !current_) return; std::vector<Polygon> polygons;
  { std::lock_guard<std::mutex> lock(mutex_); polygons = cycle_polygons_; }
  const double resolution = master.getResolution(); const Bounds window{master.getOriginX() + min_i * resolution, master.getOriginY() + min_j * resolution, master.getOriginX() + max_i * resolution, master.getOriginY() + max_j * resolution};
  const RasterSpec spec{window, resolution, static_cast<unsigned int>(max_i - min_i), static_cast<unsigned int>(max_j - min_j)}; const auto costs = rasterize(polygons, spec, profile_);
  for (unsigned int y = 0; y < spec.height; ++y) for (unsigned int x = 0; x < spec.width; ++x) { const auto legacy_cost = costs[y * spec.width + x]; const auto cost = legacy_mask_cost_to_nav2_cost(legacy_cost); const auto old = master.getCost(min_i + x, min_j + y); const auto merged = max_keepout_cost(old, cost); if (merged != old) master.setCost(min_i + x, min_j + y, merged); }
}
void VectorKeepoutLayer::reset() { std::lock_guard<std::mutex> lock(mutex_); previous_map_ = polygons_map_; current_ = true; }
}  // namespace salus_navigation_costmap
PLUGINLIB_EXPORT_CLASS(salus_navigation_costmap::VectorKeepoutLayer, nav2_costmap_2d::Layer)
