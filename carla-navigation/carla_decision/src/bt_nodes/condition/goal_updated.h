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

 private:
  geometry_msgs::PoseStamped goal_;

};





#endif //SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_DECISION_SRC_CONDITION_GOAL_UPDATED_H_
