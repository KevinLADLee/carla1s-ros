//
// Created by kevinlad on 2021/4/21.
//

#ifndef CARLA1S_ROS_CARLA_NAVIGATION_CARLA_DECISION_SRC_ACTION_STOPANDWAIT_H_
#define CARLA1S_ROS_CARLA_NAVIGATION_CARLA_DECISION_SRC_ACTION_STOPANDWAIT_H_

#include "common.h"
#include <carla_msgs/CarlaEgoVehicleControl.h>
#include <ackermann_msgs/AckermannDrive.h>

class StopAndWait : public BT::AsyncActionNode{
 public:
  StopAndWait(const std::string &name, const BT::NodeConfiguration &config);

  StopAndWait() = delete;

  static BT::PortsList providedPorts(){return {};}

  BT::NodeStatus tick() override;

  void halt() override;

 private:
  ros::NodeHandlePtr nh_ptr_;
  std::string role_name_;
  ros::Publisher cmd_pub_;
  ackermann_msgs::AckermannDrive ackermann_msg_;

};

#endif //CARLA1S_ROS_CARLA_NAVIGATION_CARLA_DECISION_SRC_ACTION_STOPANDWAIT_H_
