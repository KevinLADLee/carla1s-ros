//
// Created by kevinlad on 2021/4/21.
//

#ifndef CARLA1S_ROS_CARLA_NAVIGATION_CARLA_DECISION_SRC_ACTION_MOVE_TO_GOAL_H_
#define CARLA1S_ROS_CARLA_NAVIGATION_CARLA_DECISION_SRC_ACTION_MOVE_TO_GOAL_H_

#include "common.h"

class MoveToGoal : public BT::AsyncActionNode
{
 public:
  MoveToGoal(const std::string& name, const BT::NodeConfiguration& config)
      : BT::AsyncActionNode(name, config){}

  MoveToGoal() = delete;

  static BT::PortsList providedPorts(){return {};}

  BT::NodeStatus tick() override;

  void halt() override
  {
    _halt_requested.store(true);
  }

 private:
  std::atomic_bool _halt_requested;

};

#endif //CARLA1S_ROS_CARLA_NAVIGATION_CARLA_DECISION_SRC_ACTION_MOVE_TO_GOAL_H_
