
#include "tracking_path.h"
TrackingPath::TrackingPath(const std::string &name,
                           const std::string &action_client_name,
                           const BT::NodeConfiguration &conf) : RosActionNode(name, action_client_name, conf) {}

void TrackingPath::on_tick() {
  ROS_INFO("BT Node: TrackingPath");
  config().blackboard->get<carla_nav_msgs::Path>("path", goal_.path);
  if(goal_.path.paths.empty() || goal_.path.paths.at(0).poses.empty()) {
    on_failed_request(FailureCause::NOT_VALID_PATH);
    ROS_WARN("BT TrackingPath Node: No valid path");
  }else{
    ROS_INFO("BT TrackingPath Node: Received %ld paths", goal_.path.paths.size());
  }
}

BT::NodeStatus TrackingPath::on_result(const ResultType &res) {
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
            name, "path_tracking_action", config);
      };
  factory.registerBuilder<TrackingPath>(
      "TrackingPath", builder);
}
