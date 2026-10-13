#include <fstream>
#include <iostream>
#include <cmath>
#include <nlohmann/json.hpp>
#include <pluginlib/class_loader.hpp>
#include <nav2_core/global_planner.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_util/lifecycle_node.hpp>
#include <tf2_ros/buffer.h>

using json = nlohmann::json;
geometry_msgs::msg::PoseStamped pose(const json & p) {
  geometry_msgs::msg::PoseStamped out;
  out.header.frame_id = "map";
  out.pose.position.x = p[0]; out.pose.position.y = p[1];
  const double angle = p[2];
  out.pose.orientation.z = std::sin(angle / 2);
  out.pose.orientation.w = std::cos(angle / 2);
  return out;
}
int main(int argc, char ** argv) {
  if (argc != 3) return 2;
  std::ifstream input(argv[1]); json fixture; input >> fixture;
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions opts;
  std::vector<rclcpp::Parameter> params;
  for (auto it=fixture["planner"].begin(); it!=fixture["planner"].end(); ++it) {
    std::string key="GridBased."+it.key();
    if (it.value().is_boolean()) params.emplace_back(key,it.value().get<bool>());
    else if (it.value().is_number_integer()) params.emplace_back(key,it.value().get<int64_t>());
    else if (it.value().is_number()) params.emplace_back(key,it.value().get<double>());
    else params.emplace_back(key,it.value().get<std::string>());
  }
  opts.parameter_overrides(params);
  auto parent=std::make_shared<nav2_util::LifecycleNode>("offline_planner", "",opts);
  auto costmap=std::make_shared<nav2_costmap_2d::Costmap2DROS>("offline_costmap");
  costmap->set_parameters({rclcpp::Parameter("plugins",std::vector<std::string>{}),
    rclcpp::Parameter("global_frame","map"),rclcpp::Parameter("robot_base_frame","base_footprint"),
    rclcpp::Parameter("footprint", "[[1.05,0.38],[1.05,-0.38],[-0.12,-0.38],[-0.12,0.38]]"),
    rclcpp::Parameter("footprint_padding",0.05),rclcpp::Parameter("track_unknown_space",false)});
  if(costmap->on_configure(rclcpp_lifecycle::State())!=nav2_util::CallbackReturn::SUCCESS) return 3;
  auto grid=costmap->getCostmap();
  auto tf=std::make_shared<tf2_ros::Buffer>(parent->get_clock());
  pluginlib::ClassLoader<nav2_core::GlobalPlanner> loader("nav2_core","nav2_core::GlobalPlanner");
  json results=json::array();
  for (const auto & scenario:fixture["cases"]) {
    auto meta=scenario["map"];
    unsigned int width=meta["width"],height=meta["height"];
    grid->resizeMap(width,height,meta["resolution"],meta["origin_x"],meta["origin_y"]);
    if (scenario.contains("cost_file")) {
      std::ifstream file(scenario["cost_file"].get<std::string>(),std::ios::binary);
      file.read(reinterpret_cast<char *>(grid->getCharMap()),width*height);
      if (file.gcount()!=width*height) throw std::runtime_error("invalid costmap file");
    } else std::fill(grid->getCharMap(),grid->getCharMap()+width*height,0);
    auto planner=loader.createSharedInstance("nav2_smac_planner/SmacPlannerHybrid");
    planner->configure(parent,"GridBased",tf,costmap);
    planner->activate();
    auto start=pose(scenario["start"]);
    json segments=json::array();
    try {
      for (const auto & target:scenario["goals"]) {
        auto path=planner->createPlan(start,pose(target));
        json points=json::array(); double length=0;
        for(size_t i=0;i<path.poses.size();i++) {
          const auto & p=path.poses[i].pose;
          points.push_back({p.position.x,p.position.y,2*std::atan2(p.orientation.z,p.orientation.w)});
          if(i) length+=std::hypot(p.position.x-path.poses[i-1].pose.position.x,p.position.y-path.poses[i-1].pose.position.y);
        }
        segments.push_back({{"length",length},{"points",points}});
        if(path.poses.empty()) throw std::runtime_error("empty path");
        start=path.poses.back();
      }
      results.push_back({{"name",scenario["name"]},{"segments",segments},{"ok",true}});
    } catch(const std::exception & e) {
      results.push_back({{"name",scenario["name"]},{"error",e.what()},{"segments",segments},{"ok",false}});
    }
    planner->deactivate();planner->cleanup();planner.reset();
  }
  std::ofstream output(argv[2]);output<<results.dump(2);
  costmap->on_cleanup(rclcpp_lifecycle::State());
  costmap.reset();parent.reset();rclcpp::shutdown();
}
