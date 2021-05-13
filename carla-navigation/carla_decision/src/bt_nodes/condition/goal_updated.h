//
// Created by kevinlad on 2021/4/21.
//

#ifndef SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_DECISION_SRC_CONDITION_GOAL_UPDATED_H_
#define SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_DECISION_SRC_CONDITION_GOAL_UPDATED_H_

#include "common.h"

class GoalUpdated : public BT::ConditionNode
{
 public:
  GoalUpdated(const std::string &name, const BT::NodeConfiguration &config);

  GoalUpdated() = delete;

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return {};
  }

  void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal);

 private:
  ros::NodeHandlePtr nh_ptr_;
  ros::Subscriber goal_sub_;
  geometry_msgs::PoseStamped goal_;
  std::vector<geometry_msgs::PoseStamped> goals_;

};





#endif //SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_DECISION_SRC_CONDITION_GOAL_UPDATED_H_
