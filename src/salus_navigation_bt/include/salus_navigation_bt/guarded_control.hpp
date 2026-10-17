// Copyright 2026 SALUS maintainers
#ifndef SALUS_NAVIGATION_BT__GUARDED_CONTROL_HPP_
#define SALUS_NAVIGATION_BT__GUARDED_CONTROL_HPP_

#include <string>
#include "behaviortree_cpp_v3/control_node.h"

namespace salus_navigation_bt
{
// Keep planning and following pipelined, but synchronously halt the old
// controller when any health check asks for a stop. Candidate validation must
// not leave FollowPath running behind a planning child returning RUNNING.
class GuardedPipeline : public BT::ControlNode
{
public:
  GuardedPipeline(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ControlNode(name, config) {}

  static BT::PortsList providedPorts() {return {};}

  BT::NodeStatus tick() override
  {
    if (childrenCount() != 2) {
      throw BT::RuntimeError("GuardedPipeline needs planner and follower");
    }
    const auto planning = children_nodes_[0]->executeTick();
    bool stop = false;
    config().blackboard->get("salus_stop_path", stop);
    if (stop) {
      haltChild(1);
      following_ = false;
      config().blackboard->set("salus_stop_path", false);
    }
    if (planning == BT::NodeStatus::FAILURE) {
      haltChildren();
      following_ = false;
      return planning;
    }
    if (planning == BT::NodeStatus::RUNNING && !following_) {
      return planning;
    }
    const auto following = children_nodes_[1]->executeTick();
    following_ = following == BT::NodeStatus::RUNNING;
    if (following != BT::NodeStatus::RUNNING) {haltChildren();}
    return following;
  }

  void halt() override {BT::ControlNode::halt(); following_ = false;}

private:
  bool following_{false};
};

// Normal recovery retains its retry budget. A maneuver wait has a separate
// monotonic deadline owned by the pure policy, and never consumes/reset it.
class GuardedRecovery : public BT::ControlNode
{
public:
  GuardedRecovery(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ControlNode(name, config) {}
  static BT::PortsList providedPorts()
  {return {BT::InputPort<unsigned int>("number_of_retries", 3u, "Normal retry budget")};}
  BT::NodeStatus tick() override
  {
    if (childrenCount() != 2) {throw BT::RuntimeError("GuardedRecovery needs two children");}
    bool terminal = false;
    config().blackboard->get("salus_maneuver_terminal", terminal);
    if (terminal) {halt(); return BT::NodeStatus::FAILURE;}
    const auto status = children_nodes_[recovering_ ? 1 : 0]->executeTick();
    if (status == BT::NodeStatus::RUNNING) {return status;}
    if (!recovering_ && status == BT::NodeStatus::SUCCESS) {
      halt(); return status;
    }
    bool waiting = false;
    config().blackboard->get("salus_maneuver_wait", waiting);
    unsigned int retries = 3;
    getInput("number_of_retries", retries);
    if (!recovering_) {
      haltChild(0);
      if (!waiting && attempts_ >= retries) {halt(); return BT::NodeStatus::FAILURE;}
      recovering_ = true;
    } else {
      haltChild(1);
      recovering_ = false;
      if (!waiting) {
        if (status == BT::NodeStatus::FAILURE) {halt(); return status;}
        ++attempts_;
      }
    }
    return BT::NodeStatus::RUNNING;
  }
  void halt() override
  {
    BT::ControlNode::halt();
    recovering_ = false;
    attempts_ = 0;
    config().blackboard->set("salus_maneuver_wait", false);
    config().blackboard->set("salus_maneuver_terminal", false);
    config().blackboard->set("salus_stop_path", false);
  }

private:
  bool recovering_{false};
  unsigned int attempts_{0};
};

}  // namespace salus_navigation_bt

#endif  // SALUS_NAVIGATION_BT__GUARDED_CONTROL_HPP_
