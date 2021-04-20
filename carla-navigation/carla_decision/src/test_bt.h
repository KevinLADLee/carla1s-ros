//
// Created by kevinlad on 2021/4/14.
//

#ifndef SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_DECISION_SRC_TEST_BT_H_
#define SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_DECISION_SRC_TEST_BT_H_

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <ros/ros.h>

#include "node_types.h"

class CarlaDecision{
 public:
  CarlaDecision();

  void Tick();

  void TrafficCallback(const std_msgs::Int16::ConstPtr &msg);

//  BT::NodeStatus CheckTrafficLight() const;

 private:
  BT::Tree bt_tree_;
  BT::BehaviorTreeFactory bt_factory_;
  int traffic_light_status_;
  ros::NodeHandle nh_;

};




#endif //SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_DECISION_SRC_TEST_BT_H_
