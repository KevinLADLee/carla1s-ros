//
// Created by kevinlad on 2021/4/23.
//

#include "tracking_path.h"
TrackingPath::TrackingPath(const std::string &name,
                           const std::string &action_client_name,
                           const BT::NodeConfiguration &conf) : RosActionNode(name, action_client_name, conf) {}

void TrackingPath::on_tick() {
  config().blackboard->get<nav_msgs::Path>("path", goal_.path);
  if(goal_.path.poses.empty()) {
    on_failed_request(FailureCause::NOT_VALID_PATH);
    std::cerr << "not valid path" << std::endl;
  }else{
    std::cout << "Received "  << goal_.path.poses.size() << " waypoints" << std::endl;
  }
}

BT::NodeStatus TrackingPath::on_result(const carla_nav_msgs::TrackingPathResult_<std::allocator<void>> &res) {
  std::cout << res.error_info << std::endl;

  switch (res.error_code) {
    case 0:
      return BT::NodeStatus::IDLE;
    case 1:
      return BT::NodeStatus::RUNNING;
    case 4:
      return BT::NodeStatus::SUCCESS;
    case 5:
      return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::FAILURE;
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
      [](const std::string & name, const BT::NodeConfiguration & config)
      {
        return std::make_unique<TrackingPath>(
            name, "tracking_path_action", config);
      };
  factory.registerBuilder<TrackingPath>(
      "TrackingPath", builder);
}