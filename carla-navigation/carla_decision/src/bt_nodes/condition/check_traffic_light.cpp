
#ifndef SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_DECISION_SRC_CONDITION_CHECK_TRAFFIC_LIGHT_H_
#define SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_DECISION_SRC_CONDITION_CHECK_TRAFFIC_LIGHT_H_

#include "check_traffic_light.h"

  CheckTrafficLight::CheckTrafficLight(const std::string &condition_name,
                    const BT::NodeConfiguration &conf) : BT::ConditionNode(condition_name, conf){
    traffic_sub_ = nh.subscribe<std_msgs::Int16>("/traffic_light_stats", 10,
                                                 boost::bind(&CheckTrafficLight::TrafficCallback, this, _1));
  };

  void CheckTrafficLight::TrafficCallback(const std_msgs::Int16::ConstPtr &msg){
    traffic_light_status_ = msg->data;
  }

  static BT::PortsList CheckTrafficLight::providedPorts()
  {
    return { BT::InputPort<std::string>("traffic_light_status") };
  }

  BT::NodeStatus CheckTrafficLight::tick() override
  {
    switch (traffic_light_status_) {
      case TrafficLightStats::RED:
      case TrafficLightStats::YELLOW:
        return BT::NodeStatus::FAILURE;
        break;
      case TrafficLightStats::GREEN:
      default:
        return BT::NodeStatus::SUCCESS;
    }
  }

#endif //SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_DECISION_SRC_CONDITION_CHECK_TRAFFIC_LIGHT_H_
