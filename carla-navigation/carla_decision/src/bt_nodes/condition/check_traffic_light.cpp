#include "check_traffic_light.h"

CheckTrafficLight::CheckTrafficLight(const std::string &condition_name,
                                     const BT::NodeConfiguration &conf) : BT::ConditionNode(condition_name, conf){

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
