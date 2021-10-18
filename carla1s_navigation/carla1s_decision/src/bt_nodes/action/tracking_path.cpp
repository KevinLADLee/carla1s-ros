
#include "tracking_path.h"
TrackingPath::TrackingPath(const std::string &name,
                           const std::string &action_client_name,
                           const BT::NodeConfiguration &conf) : RosActionNode(name, action_client_name, conf) {}

void TrackingPath::on_tick() {
//  ROS_INFO("BT Node: TrackingPath");

  // Get path from behavior tree's ports
  auto path_port = getInput<carla1s_msgs::PathArray>("path");
  if(!path_port){
    setStatus(BT::NodeStatus::FAILURE);
    auto goal_port = getInput<geometry_msgs::PoseStamped>("goal");
    if(!goal_port){
      ROS_WARN("BT TrackingPath Node: No goal");
      return;
    }else{
      ROS_WARN("BT TrackingPath Node: No valid path");
      config().blackboard->set<bool>("need_update_path", true);
    }
    return;
  }else{
    goal_.path_array = path_port.value();
    goal_.path_updated = config().blackboard->get<bool>("path_updated");
  }
  if(goal_.path_array.paths.empty()) {
    on_failed_request(FailureCause::NOT_VALID_PATH);
    ROS_WARN("BT TrackingPath Node: No valid path");
  }else{
    if(goal_.path_array.paths.at(0).poses.empty()){
      on_failed_request(FailureCause::NOT_VALID_PATH);
      ROS_WARN("BT TrackingPath Node: No valid path");
    }
    if(goal_.path_updated) {
      ROS_INFO("BT TrackingPath Node: Received %ld paths", goal_.path_array.paths.size());
    }
  }

  // Get target speed from behavior tree's ports
  auto target_speed_port = getInput<double>("target_speed");
  if(!target_speed_port){
    throw BT::RuntimeError("missing required input [target_speed]: ", target_speed_port.error());
  }else{
    goal_.target_speed = target_speed_port.value();
  }

}

BT::NodeStatus TrackingPath::on_result(const ResultType &res) {
  switch (res.error_code) {
    case NodeState::IDLE:
    case NodeState::SUCCESS:
      config().blackboard->set<bool>("goal_received", false);
      config().blackboard->set<bool>("path_updated", false);
      return BT::NodeStatus::SUCCESS;
    case NodeState::FAILURE:
      return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus TrackingPath::on_failed_request(BT::RosActionNode<carla1s_msgs::PathTrackingAction>::FailureCause failure) {
  ROS_WARN("Track path failed! Replan now");
  config().blackboard->set<bool>("need_update_path", true);
  return RosActionNode::on_failed_request(failure);
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
