
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

  void TrafficCallback(const std_msgs::Int16::ConstPtr &msg);

  static BT::PortsList providedPorts(){return{};};

  BT::NodeStatus tick() override;
  
 private:
  ros::NodeHandle nh;
  int traffic_light_status_ = TrafficLightStats::GREEN;
  ros::Subscriber traffic_sub_;
};

#endif //SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_DECISION_SRC_CONDITION_CHECK_TRAFFIC_LIGHT_H_
