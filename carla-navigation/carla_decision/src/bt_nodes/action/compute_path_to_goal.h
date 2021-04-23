
#ifndef SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_DECISION_SRC_ACTION_COMPUTE_PATH_TO_GOAL_H_
#define SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_DECISION_SRC_ACTION_COMPUTE_PATH_TO_GOAL_H_

#include "common.h"
#include <carla_nav_msgs/PathPlannerAction.h>

class ComputePathToGoal : public BT::RosActionNode<carla_nav_msgs::PathPlannerAction> {
 public:
  ComputePathToGoal(const std::string &name, const std::string &action_client_name, const BT::NodeConfiguration &conf);

  ComputePathToGoal() = delete;

  void on_tick() override;

  static BT::PortsList providedPorts(){return {};};

};

#endif //SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_DECISION_SRC_ACTION_COMPUTE_PATH_TO_GOAL_H_
