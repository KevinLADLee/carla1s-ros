#include "check_traffic_light.h"

CheckTrafficLight::CheckTrafficLight(const std::string &condition_name,
                                     const BT::NodeConfiguration &conf) : BT::ConditionNode(condition_name, conf){
  traffic_sub_ = nh.subscribe<std_msgs::Int16>("/traffic_light_stats", 10,
                                               boost::bind(&CheckTrafficLight::TrafficCallback, this, _1));
};

void CheckTrafficLight::TrafficCallback(const std_msgs::Int16::ConstPtr &msg){
  traffic_light_status_ = msg->data;
}

BT::NodeStatus CheckTrafficLight::tick()
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

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<CheckTrafficLight>("CheckTrafficLight");
}
