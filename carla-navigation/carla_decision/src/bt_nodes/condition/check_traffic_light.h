
#ifndef CARLA1S_ROS_CARLA_NAVIGATION_CARLA_DECISION_SRC_CONDITION_CHECK_TRAFFIC_LIGHT_H_
#define CARLA1S_ROS_CARLA_NAVIGATION_CARLA_DECISION_SRC_CONDITION_CHECK_TRAFFIC_LIGHT_H_

#include "carla_decision_common.h"


class CheckTrafficLight : public BT::ConditionNode{
 public:
  CheckTrafficLight(const std::string &condition_name,
                    const BT::NodeConfiguration &conf);

  static BT::PortsList providedPorts(){return{};};

  BT::NodeStatus tick() override;
  
 private:
  bool traffic_light_passable_ = true;

};

#endif //CARLA1S_ROS_CARLA_NAVIGATION_CARLA_DECISION_SRC_CONDITION_CHECK_TRAFFIC_LIGHT_H_
