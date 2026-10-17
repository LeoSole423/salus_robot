// Copyright 2026 SALUS maintainers
#include <functional>
#include <string>
#include "gtest/gtest.h"
#include "behaviortree_cpp_v3/action_node.h"
#include "salus_navigation_bt/guarded_control.hpp"

class Probe : public BT::ActionNodeBase
{
public:
  Probe(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ActionNodeBase(name, config) {}
  BT::NodeStatus answer{BT::NodeStatus::RUNNING};
  std::function<void()> before;
  int stops{0};
  int ticks{0};
  BT::NodeStatus tick() override {return step();}
  void halt() override
  {
    if (status() == BT::NodeStatus::RUNNING) {
      ++stops;
    }
    setStatus(BT::NodeStatus::IDLE);
  }
  BT::NodeStatus step() {++ticks; if (before) {before();} return answer;}
};

TEST(GuardedPipeline, HaltsOldFollowerEvenWhilePlannerIsRunning)
{
  BT::NodeConfiguration config;
  config.blackboard = BT::Blackboard::create();
  salus_navigation_bt::GuardedPipeline pipeline("pipeline", config);
  Probe planner("planner", config), follower("follower", config);
  pipeline.addChild(&planner); pipeline.addChild(&follower);
  planner.answer = BT::NodeStatus::SUCCESS;
  EXPECT_EQ(pipeline.executeTick(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(follower.ticks, 1);
  planner.halt();
  planner.answer = BT::NodeStatus::RUNNING;
  EXPECT_EQ(pipeline.executeTick(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(follower.ticks, 2);  // Healthy replans retain normal pipelining.
  planner.before = [&]() {config.blackboard->set("salus_stop_path", true);};
  EXPECT_EQ(pipeline.executeTick(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(follower.stops, 1);
  EXPECT_EQ(follower.ticks, 2);  // No old path tick after rejection.
  planner.before = nullptr;
  planner.answer = BT::NodeStatus::SUCCESS;
  EXPECT_EQ(pipeline.executeTick(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(follower.ticks, 3);
  pipeline.halt();
  EXPECT_EQ(follower.stops, 2);
}

TEST(GuardedRecovery, ManeuverWaitDoesNotConsumeNormalBudgetAndTerminalHalts)
{
  BT::NodeConfiguration config;
  config.blackboard = BT::Blackboard::create();
  config.blackboard->set("salus_maneuver_wait", true);
  salus_navigation_bt::GuardedRecovery recovery("recovery", config);
  Probe primary("primary", config), retry("retry", config);
  recovery.addChild(&primary); recovery.addChild(&retry);
  primary.answer = BT::NodeStatus::FAILURE;
  retry.answer = BT::NodeStatus::SUCCESS;
  for (int i = 0; i < 20; ++i) {
    EXPECT_EQ(recovery.executeTick(), BT::NodeStatus::RUNNING);
  }
  config.blackboard->set("salus_maneuver_terminal", true);
  EXPECT_EQ(recovery.executeTick(), BT::NodeStatus::FAILURE);
}

TEST(GuardedRecovery, OrdinaryFailuresRetainThreeRetryBudget)
{
  BT::NodeConfiguration config;
  config.blackboard = BT::Blackboard::create();
  salus_navigation_bt::GuardedRecovery recovery("recovery", config);
  Probe primary("primary", config), retry("retry", config);
  recovery.addChild(&primary); recovery.addChild(&retry);
  primary.answer = BT::NodeStatus::FAILURE;
  retry.answer = BT::NodeStatus::SUCCESS;
  for (int i = 0; i < 6; ++i) {
    EXPECT_EQ(recovery.executeTick(), BT::NodeStatus::RUNNING);
  }
  EXPECT_EQ(recovery.executeTick(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(retry.ticks, 3);
}
