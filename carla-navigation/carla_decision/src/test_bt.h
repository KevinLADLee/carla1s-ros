//
// Created by kevinlad on 2021/4/14.
//

#ifndef SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_DECISION_SRC_TEST_BT_H_
#define SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_DECISION_SRC_TEST_BT_H_

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <behaviortree_cpp_v3/utils/shared_library.h>
#include <ros/ros.h>

#include "node_base/bt_action_node.h"
#include "action/move_to_goal.h"
#include "action/StopAndWait.h"
#include "condition/check_traffic_light.h"
#include "condition/goal_updated.h"

#include <iostream>
#include <fstream>

class CarlaDecision{
 public:
  CarlaDecision();

  bool LoadBehaviorTree(const std::string & filename);

  void Tick();

  void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr& goal);

 private:
  BT::Tree bt_tree_;
  BT::Blackboard::Ptr bt_blackboard_;
  BT::BehaviorTreeFactory bt_factory_;
  int traffic_light_status_;
  ros::NodeHandle nh_;
  std::shared_ptr<BT::PublisherZMQ> publisher_zmq_ptr_;

  ros::Subscriber goal_sub_;

};




#endif //SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_DECISION_SRC_TEST_BT_H_
