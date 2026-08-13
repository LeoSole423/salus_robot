// Copyright 2026 SALUS maintainers

#include <memory>
#include <string>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "nav2_behavior_tree/bt_service_node.hpp"
#include "nav2_msgs/srv/is_path_valid.hpp"
#include "nav_msgs/msg/path.hpp"

namespace salus_navigation_bt
{
class PathHealthCondition
  : public nav2_behavior_tree::BtServiceNode<nav2_msgs::srv::IsPathValid>
{
public:
  PathHealthCondition(const std::string & name, const BT::NodeConfiguration & config)
  : BtServiceNode<nav2_msgs::srv::IsPathValid>(name, config, "/path_health/is_path_valid") {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({BT::InputPort<nav_msgs::msg::Path>("path")});
  }

  void on_tick() override
  {
    nav_msgs::msg::Path path;
    if (!getInput("path", path)) {
      should_send_request_ = false;
      return;
    }
    request_->path = path;
  }

  BT::NodeStatus on_completion(
    std::shared_ptr<nav2_msgs::srv::IsPathValid::Response> response)
  override
  {
    return response->is_valid ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }
};

class CopyPath : public BT::SyncActionNode
{
public:
  CopyPath(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("input_path"),
      BT::OutputPort<nav_msgs::msg::Path>("output_path")};
  }

  BT::NodeStatus tick() override
  {
    nav_msgs::msg::Path path;
    if (!getInput("input_path", path)) {
      return BT::NodeStatus::FAILURE;
    }
    setOutput("output_path", path);
    return BT::NodeStatus::SUCCESS;
  }
};
}  // namespace salus_navigation_bt

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<salus_navigation_bt::PathHealthCondition>("IsPathHealthValid");
  factory.registerNodeType<salus_navigation_bt::CopyPath>("CopyPath");
}
