
#ifndef SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_DECISION_SRC_CONDITION_CHECK_TRAFFIC_LIGHT_H_
#define SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_DECISION_SRC_CONDITION_CHECK_TRAFFIC_LIGHT_H_

#include "common.h"


class CheckTrafficLight : public BT::ConditionNode{
 public:
  CheckTrafficLight(const std::string &condition_name,
                    const BT::NodeConfiguration &conf);

  static BT::PortsList providedPorts(){return{};};

  BT::NodeStatus tick() override;
  
 private:
  bool traffic_light_passable_ = true;

};

#endif //SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_DECISION_SRC_CONDITION_CHECK_TRAFFIC_LIGHT_H_
