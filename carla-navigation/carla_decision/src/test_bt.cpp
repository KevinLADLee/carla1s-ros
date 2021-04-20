//
// Created by kevinlad on 2021/4/14.
//

#include "test_bt.h"
#include "node_types.h"


CarlaDecision::CarlaDecision() {


  bt_factory_.registerNodeType<MoveToGoal>("MoveToGoal");
  bt_factory_.registerSimpleAction("StopAndWait", std::bind(StopAndWait));
  bt_factory_.registerNodeType<CheckTrafficLight>("CheckTrafficLight");
  bt_tree_ = bt_factory_.createTreeFromFile("/home/kevinlad/carla/overtaking_behavior_tree/ros1_ws/src/carla-ros-bridge/carla-navigation/carla_decision/trees/goal_behavior.xml");


}

void CarlaDecision::Tick() {
  ros::Rate rate(10);
  while (ros::ok()){
    ros::spinOnce();
    bt_tree_.tickRoot();
    rate.sleep();
  }
}

//BT::NodeStatus CarlaDecision::CheckTrafficLight() const {
//  if(traffic_light_){
//    return BT::NodeStatus::SUCCESS;
//  } else {
//    return BT::NodeStatus::FAILURE;
//  }
//}
