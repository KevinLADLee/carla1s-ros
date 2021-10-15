
#include "detect_vehicle.h"

namespace carla1s_decision {

DetectVehicle::DetectVehicle(const std::string &name, const BT::NodeConfiguration &config) : AsyncActionNode(name,
                                                                                                             config) {}

BT::NodeStatus DetectVehicle::tick() {
  auto safe_speed = config().blackboard->get<double>("collision_avoid_speed");
  double target_speed = 0.0;
  if (safe_speed < 0) {
    getInput<double>("target_speed", target_speed);
    setOutput("target_speed", target_speed);
    return BT::NodeStatus::SUCCESS;
  } else {
    getInput<double>("target_speed", target_speed);
    target_speed = std::min<double>(safe_speed, target_speed);
    ROS_INFO("Front Vehicle Detected! Limit speed to %f", target_speed);
    setOutput("target_speed", target_speed);
  }
  return BT::NodeStatus::SUCCESS;
}

}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<carla1s_decision::DetectVehicle>("DetectVehicle");
}