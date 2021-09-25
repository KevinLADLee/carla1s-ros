
#include "tracking_path.h"
TrackingPath::TrackingPath(const std::string &name,
                           const std::string &action_client_name,
                           const BT::NodeConfiguration &conf) : RosActionNode(name, action_client_name, conf) {}

void TrackingPath::on_tick() {
  ROS_INFO("BT Node: TrackingPath");
  auto path_port = getInput<carla1s_msgs::Path>("path");
  if(!path_port){
    throw BT::RuntimeError("missing required input [path]: ", path_port.error());
  }else{
    goal_.path = path_port.value();
  }
  if(goal_.path.paths.empty()) {
    on_failed_request(FailureCause::NOT_VALID_PATH);
    ROS_WARN("BT TrackingPath Node: No valid path");
  }else{
    if(goal_.path.paths.at(0).poses.empty()){
      on_failed_request(FailureCause::NOT_VALID_PATH);
      ROS_WARN("BT TrackingPath Node: No valid path");
    }
    ROS_INFO("BT TrackingPath Node: Received %ld paths", goal_.path.paths.size());
  }
}

BT::NodeStatus TrackingPath::on_result(const ResultType &res) {
  switch (res.error_code) {
    case NodeState::IDLE:
      return BT::NodeStatus::IDLE;
    case NodeState::SUCCESS:
      config().blackboard->set<bool>("goal_received", false);
      return BT::NodeStatus::SUCCESS;
    case NodeState::FAILURE:
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
