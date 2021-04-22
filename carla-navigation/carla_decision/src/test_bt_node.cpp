//
// Created by kevinlad on 2021/4/14.
//

#include "test_bt.h"

int main(int argc, char** argv){

  ros::init(argc, argv, "carla_decision");
  CarlaDecision carla_tree_;
  carla_tree_.LoadBehaviorTree("/home/kevinlad/carla/overtaking_behavior_tree/ros1_ws/src/carla-ros-bridge/carla-navigation/carla_decision/trees/goal_behavior.xml");
  carla_tree_.Tick();

  return 0;
}