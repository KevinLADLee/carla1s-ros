
#ifndef CARLA1S_ROS_CARLA_NAVIGATION_CARLA_DECISION_SRC_ACTION_COMPUTE_PATH_TO_GOAL_H_
#define CARLA1S_ROS_CARLA_NAVIGATION_CARLA_DECISION_SRC_ACTION_COMPUTE_PATH_TO_GOAL_H_

#include "carla_decision_common.h"
#include <carla_nav_msgs/Path.h>
#include <carla_nav_msgs/PathPlannerAction.h>
#include <actionlib/client/simple_action_client.h>

class ComputePathToGoal : public BT::RosActionNode<carla_nav_msgs::PathPlannerAction> {
 public:
  ComputePathToGoal(const std::string &name, const std::string &action_client_name, const BT::NodeConfiguration &conf);

  ComputePathToGoal() = delete;

  void on_tick() override;

  BT::NodeStatus on_result(const ResultType &res) override;

  static BT::PortsList providedPorts()
  {
    return {
        BT::InputPort<geometry_msgs::PoseStamped>("goal"),
        BT::InputPort<std::string>("path_planner_id"),
        BT::OutputPort<carla_nav_msgs::Path>("path")
    };
  };
};

#endif //CARLA1S_ROS_CARLA_NAVIGATION_CARLA_DECISION_SRC_ACTION_COMPUTE_PATH_TO_GOAL_H_
