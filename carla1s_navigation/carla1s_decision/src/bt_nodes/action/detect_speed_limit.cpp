
#include "detect_speed_limit.h"

namespace carla1s_decision {

DetectSpeedLimit::DetectSpeedLimit(const std::string &name, const BT::NodeConfiguration &config) : BT::AsyncActionNode(name,
                                                                                                                       config) {}
BT::NodeStatus DetectSpeedLimit::tick() {
  // TODO: Get speed limit from fake perception
  getInput<double>("default_speed", default_speed_);
  setOutput("target_speed", default_speed_);
//  setOutput("target_speed", min(speed_limit, default_speed);
  return BT::NodeStatus::SUCCESS;
}

}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<carla1s_decision::DetectSpeedLimit>("DetectSpeedLimit");
}