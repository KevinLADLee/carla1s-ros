
#include "goal_updated.h"

BT::NodeStatus GoalUpdated::tick()
{
  if (status() == BT::NodeStatus::IDLE) {
    config().blackboard->get<std::vector<geometry_msgs::PoseStamped>>("goals", goals_);
    config().blackboard->get<geometry_msgs::PoseStamped>("goal", goal_);
    return BT::NodeStatus::FAILURE;
  }

  std::vector<geometry_msgs::PoseStamped> current_goals;
  config().blackboard->get<std::vector<geometry_msgs::PoseStamped>>("goals", current_goals);
  geometry_msgs::PoseStamped current_goal;
  config().blackboard->get<geometry_msgs::PoseStamped>("goal", current_goal);

  if (goal_ != current_goal || goals_ != current_goals) {
    goal_ = current_goal;
    goals_ = current_goals;
    DLOG_INFO( "Goal Updated!");
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::FAILURE;
}

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<GoalUpdated>("GoalUpdated");
}
