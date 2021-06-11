#include "check_traffic_light.h"

CheckTrafficLight::CheckTrafficLight(const std::string &condition_name,
                                     const BT::NodeConfiguration &conf) : BT::ConditionNode(condition_name, conf){}

BT::NodeStatus CheckTrafficLight::tick()
{
  config().blackboard->get<bool>("traffic_light_passable", traffic_light_passable_);
  if(!traffic_light_passable_){
    return BT::NodeStatus::FAILURE;
  } else{
    return BT::NodeStatus::SUCCESS;
  }
}

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<CheckTrafficLight>("CheckTrafficLight");
}
