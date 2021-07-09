//
// Created by kevinlad on 2021/6/10.
//

#ifndef CARLA1S_ROS_CARLA_NAVIGATION_CARLA_DECISION_SRC_BT_NODES_ACTION_PARKING_H_
#define CARLA1S_ROS_CARLA_NAVIGATION_CARLA_DECISION_SRC_BT_NODES_ACTION_PARKING_H_

#include "common.h"
#include <carla_nav_msgs/ParkingPlannerAction.h>

class Parking : public BT::RosActionNode<carla_nav_msgs::ParkingPlannerAction> {
  Parking(const std::string &name, const std::string &action_client_name, const BT::NodeConfiguration &conf);

  Parking() = delete;

  void on_tick() override;

  static BT::PortsList providedPorts(){return {};};

  BT::NodeStatus on_result(const ResultType &res) override;

};

#endif //CARLA1S_ROS_CARLA_NAVIGATION_CARLA_DECISION_SRC_BT_NODES_ACTION_PARKING_H_
