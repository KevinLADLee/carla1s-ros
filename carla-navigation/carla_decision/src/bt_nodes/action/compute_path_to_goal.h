
#ifndef SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_DECISION_SRC_ACTION_COMPUTE_PATH_TO_GOAL_H_
#define SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_DECISION_SRC_ACTION_COMPUTE_PATH_TO_GOAL_H_

#include "common.h"
#include <carla_nav_msgs/PathPlannerAction.h>
#include <actionlib/client/simple_action_client.h>

class ComputePathToGoal : public BT::RosActionNode<carla_nav_msgs::PathPlannerAction> {
 public:
  ComputePathToGoal(const std::string &name, const std::string &action_client_name, const BT::NodeConfiguration &conf);

  ComputePathToGoal() = delete;

  void on_tick() override;

  BT::NodeStatus onResult(const ResultType &res) override;

  static BT::PortsList providedPorts()
  {
    return {
        BT::OutputPort<nav_msgs::Path>("path", "Path created by ComputePathToGoal node"),
        BT::InputPort<geometry_msgs::PoseStamped>("goal", "Destination to plan to"),
        BT::InputPort<geometry_msgs::PoseStamped>("start", "Start pose of the path if overriding current robot pose"),
    };
  };
};

#endif //SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_DECISION_SRC_ACTION_COMPUTE_PATH_TO_GOAL_H_
