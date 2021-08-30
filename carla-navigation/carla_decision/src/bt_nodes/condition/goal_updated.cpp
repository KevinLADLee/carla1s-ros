
#include "goal_updated.h"

GoalUpdated::GoalUpdated(const std::string &name, const BT::NodeConfiguration &config) : ConditionNode(name, config) {
  goal_received_ = false;
}

BT::NodeStatus GoalUpdated::tick()
{
  config().blackboard->get<bool>("goal_received", goal_received_);
  if(!goal_received_){
    return BT::NodeStatus::FAILURE;
  }

  if (status() == BT::NodeStatus::IDLE) {
    config().blackboard->get<geometry_msgs::PoseStamped>("goal", goal_);
    return BT::NodeStatus::FAILURE;
  }

  geometry_msgs::PoseStamped current_goal;
  config().blackboard->get<geometry_msgs::PoseStamped>("goal", current_goal);

  if (goal_ != current_goal) {
    goal_ = current_goal;
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<GoalUpdated>("GoalUpdated");
}
