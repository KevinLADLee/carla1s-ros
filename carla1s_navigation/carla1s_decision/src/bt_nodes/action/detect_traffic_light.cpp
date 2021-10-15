
#include "detect_traffic_light.h"

namespace carla1s_decision {

DetectTrafficLight::DetectTrafficLight(const std::string &name,
                                       const BT::NodeConfiguration &conf) : AsyncActionNode(name, conf) {

}

BT::NodeStatus DetectTrafficLight::tick() {
  config().blackboard->get<bool>("traffic_light_passable", traffic_light_passable_);

  if (!traffic_light_passable_) {
    ROS_INFO("TrafficLight Red! Stop and wait...");
    setOutput("target_speed", 0.0);
  }else{
    double target_speed;
    getInput<double>("target_speed", target_speed);
    setOutput("target_speed", target_speed);
  }
  return BT::NodeStatus::SUCCESS;
}

}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<carla1s_decision::DetectTrafficLight>("DetectTrafficLight");
}