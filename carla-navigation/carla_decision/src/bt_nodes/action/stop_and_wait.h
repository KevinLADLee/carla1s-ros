//
// Created by kevinlad on 2021/4/21.
//

#ifndef SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_DECISION_SRC_ACTION_STOPANDWAIT_H_
#define SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_DECISION_SRC_ACTION_STOPANDWAIT_H_

#include "common.h"

class StopAndWait : public BT::AsyncActionNode{
 public:
  StopAndWait(const std::string &name, const BT::NodeConfiguration &config);

  StopAndWait() = delete;

  static BT::PortsList providedPorts(){return {};}

  BT::NodeStatus tick() override;

};

#endif //SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_DECISION_SRC_ACTION_STOPANDWAIT_H_
