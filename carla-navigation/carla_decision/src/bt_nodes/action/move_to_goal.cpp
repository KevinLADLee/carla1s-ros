//
// Created by kevinlad on 2021/4/21.
//

#include "move_to_goal.h"

BT::NodeStatus MoveToGoal::tick()
{
// the output may change at each tick(). Here we keep it simple.

  ros::Rate r(1);
//    while (ros::ok()) {
  std::cout << "MoveToGoal" << std::endl;
  r.sleep();
//    }
  return BT::NodeStatus::SUCCESS;
}