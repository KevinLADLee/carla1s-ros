
#include "goal_reached.h"

namespace carla1s_decision{

GoalReached::GoalReached(const std::string &name, const BT::NodeConfiguration &config) : ConditionNode(name, config) {

}

BT::NodeStatus GoalReached::tick() {

  auto goal_port = getInput<geometry_msgs::PoseStamped>("goal");
  if(!goal_port){
    ROS_WARN("GoalReached: No Valid Goal!");
    return BT::NodeStatus::FAILURE;
  }

  setOutput("goal", goal_port.value());
  config().blackboard->set("goal", goal_port.value());
  config().blackboard->set("goal_received", true);
  if(first_) {
    config().blackboard->set<bool>("need_update_path", true);
    first_ = false;
  }

  auto goal_pose = goal_port.value();
  auto odom = config().blackboard->get<nav_msgs::Odometry>("odom");

  double radius = 0.0;
  getInput<double>("goal_radius", radius);
  auto dx = odom.pose.pose.position.x - goal_pose.pose.position.x;
  auto dy = odom.pose.pose.position.y - goal_pose.pose.position.y;
  auto dist = std::sqrt(dx * dx + dy * dy);

  ROS_INFO("(%f, %f) , (%f, %f)", odom.pose.pose.position.x,
           odom.pose.pose.position.y,
           goal_pose.pose.position.x,
           goal_pose.pose.position.y);

  if(dist < radius){
    std::cout << "success" << std::endl;
    return BT::NodeStatus::SUCCESS;
  }else{
    return BT::NodeStatus::FAILURE;
  }

}

}

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<carla1s_decision::GoalReached>("GoalReached");
}
