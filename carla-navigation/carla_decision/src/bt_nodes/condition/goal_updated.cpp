
#include "goal_updated.h"

GoalUpdated::GoalUpdated(const std::string &name, const BT::NodeConfiguration &config) : ConditionNode(name, config) {
  config.blackboard->get<ros::NodeHandlePtr>("node_handler", nh_ptr_);
  goal_sub_ = nh_ptr_->subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, &GoalUpdated::GoalCallback, this);
}

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
    config().blackboard->set<bool>("goal_updated", true);
    DLOG_INFO( "Goal Updated!");
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

void GoalUpdated::GoalCallback(const geometry_msgs::PoseStampedConstPtr &goal) {
  config().blackboard->set<geometry_msgs::PoseStamped>("goal", *goal);
  setStatus(BT::NodeStatus::SUCCESS);
}

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<GoalUpdated>("GoalUpdated");
}
