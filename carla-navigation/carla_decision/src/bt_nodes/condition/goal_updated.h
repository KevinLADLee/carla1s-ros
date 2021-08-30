
#ifndef CARLA1S_ROS_CARLA_NAVIGATION_CARLA_DECISION_SRC_CONDITION_GOAL_UPDATED_H_
#define CARLA1S_ROS_CARLA_NAVIGATION_CARLA_DECISION_SRC_CONDITION_GOAL_UPDATED_H_

#include "carla_decision_common.h"

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
  bool goal_received_;

};





#endif //CARLA1S_ROS_CARLA_NAVIGATION_CARLA_DECISION_SRC_CONDITION_GOAL_UPDATED_H_
