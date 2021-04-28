
#ifndef SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_DECISION_SRC_CONDITION_CHECK_TRAFFIC_LIGHT_H_
#define SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_DECISION_SRC_CONDITION_CHECK_TRAFFIC_LIGHT_H_

#include "common.h"


class CheckTrafficLight : public BT::ConditionNode{
  enum TrafficLightStats{
    RED = 0,
    YELLOW = 1,
    GREEN = 2
  };

 public:
  CheckTrafficLight(const std::string &condition_name,
                    const BT::NodeConfiguration &conf);

  static BT::PortsList providedPorts(){return{};};

  BT::NodeStatus tick() override;
  
 private:
  int traffic_light_status_ = TrafficLightStats::GREEN;

};

#endif //SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_DECISION_SRC_CONDITION_CHECK_TRAFFIC_LIGHT_H_
