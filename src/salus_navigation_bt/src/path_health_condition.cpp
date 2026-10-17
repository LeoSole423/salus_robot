// Copyright 2026 SALUS maintainers

#include <memory>
#include <string>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "nav2_behavior_tree/bt_service_node.hpp"
#include "nav_msgs/msg/path.hpp"
#include "salus_navigation_bt/guarded_control.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "salus_interfaces/msg/path_health.hpp"
#include "salus_interfaces/srv/evaluate_path_health.hpp"

namespace salus_navigation_bt
{
class PathHealthCondition
  : public nav2_behavior_tree::BtServiceNode<salus_interfaces::srv::EvaluatePathHealth>
{
public:
  PathHealthCondition(const std::string & name, const BT::NodeConfiguration & config)
  : BtServiceNode<salus_interfaces::srv::EvaluatePathHealth>(
      name, config, "/path_health/evaluate") {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts(
      {
        BT::InputPort<nav_msgs::msg::Path>("path"),
        BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),
        BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>("goals"),
        BT::InputPort<bool>("data_only", false, "Only unavailable data"),
        BT::InputPort<unsigned int>("context"),
        BT::InputPort<unsigned int>("expected_state")});
  }

  BT::NodeStatus tick() override
  {
    const auto status = BtServiceNode::tick();
    if (status == BT::NodeStatus::FAILURE && !response_received_) {
      config().blackboard->set("salus_stop_path", true);
    }
    return status;
  }

  void on_tick() override
  {
    response_received_ = false;
    nav_msgs::msg::Path path;
    if (!getInput("path", path)) {
      should_send_request_ = false;
      return;
    }
    request_->path = path;
    request_->goals.clear();
    if (!getInput("goals", request_->goals)) {
      geometry_msgs::msg::PoseStamped goal;
      if (getInput("goal", goal)) {request_->goals.push_back(goal);}
    }
    unsigned int context = salus_interfaces::srv::EvaluatePathHealth::Request::ACTIVE;
    unsigned int expected_state = salus_interfaces::msg::PathHealth::KEEP_PATH;
    getInput("context", context);
    getInput("expected_state", expected_state);
    request_->context = static_cast<uint8_t>(context);
    expected_state_ = static_cast<uint8_t>(expected_state);
  }

  BT::NodeStatus on_completion(
    std::shared_ptr<salus_interfaces::srv::EvaluatePathHealth::Response> response)
  override
  {
    response_received_ = true;
    const auto reason = response->health.reason;
    if (reason.rfind("maneuver:", 0) == 0) {
      config().blackboard->set("salus_maneuver_wait", true);
      config().blackboard->set(
        "salus_maneuver_terminal",
        reason.rfind("maneuver:NEEDS_OPERATOR:", 0) == 0);
    } else {
      const bool active = request_->context ==
        salus_interfaces::srv::EvaluatePathHealth::Request::ACTIVE;
      if (active && response->health.state == salus_interfaces::msg::PathHealth::KEEP_PATH) {
        config().blackboard->set("salus_maneuver_wait", false);
        config().blackboard->set("salus_maneuver_terminal", false);
      }
    }
    if (response->health.state == salus_interfaces::msg::PathHealth::STOP_AND_WAIT) {
      config().blackboard->set("salus_stop_path", true);
    }
    bool data_only = false;
    getInput("data_only", data_only);
    if (data_only && response->health.reason.rfind("maneuver:", 0) == 0) {
      return BT::NodeStatus::FAILURE;
    }
    return response->health.state == expected_state_ ?
           BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

private:
  bool response_received_{false};
  uint8_t expected_state_{salus_interfaces::msg::PathHealth::KEEP_PATH};
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
  factory.registerNodeType<salus_navigation_bt::GuardedRecovery>("GuardedRecovery");
  factory.registerNodeType<salus_navigation_bt::GuardedPipeline>("GuardedPipeline");
  factory.registerNodeType<salus_navigation_bt::CopyPath>("CopyPath");
}
