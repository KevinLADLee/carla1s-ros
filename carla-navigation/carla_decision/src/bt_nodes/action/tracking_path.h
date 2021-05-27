//
// Created by kevinlad on 2021/4/23.
//

#ifndef SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_DECISION_SRC_BT_NODES_ACTION_TRACKING_PATH_H_
#define SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_DECISION_SRC_BT_NODES_ACTION_TRACKING_PATH_H_

#include "common.h"

#include <carla_nav_msgs/PathTrackingAction.h>

class TrackingPath : public BT::RosActionNode<carla_nav_msgs::PathTrackingAction> {
 public:
  TrackingPath(const std::string &name, const std::string &action_client_name, const BT::NodeConfiguration &conf);
  TrackingPath() = delete;

  void on_tick() override;

  static BT::PortsList providedPorts(){return {
//        BT::InputPort<nav_msgs::Path>("path", "Path created by ComputePathToGoal node"),
  };}
  BT::NodeStatus on_result(const ResultType &res) override;;
};

#endif //SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_DECISION_SRC_BT_NODES_ACTION_TRACKING_PATH_H_
