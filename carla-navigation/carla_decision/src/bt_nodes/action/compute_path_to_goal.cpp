
#include "compute_path_to_goal.h"

ComputePathToGoal::ComputePathToGoal(const std::string &name, const std::string &action_client_name, const BT::NodeConfiguration &conf)
    : RosActionNode(name, action_client_name, conf) {}

void ComputePathToGoal::on_tick() {
  config().blackboard->get<bool>("goal_updated", goal_updated_);
  config().blackboard->get<geometry_msgs::PoseStamped>("goal", goal_.goal);
  goal_.role_name = "ego_vehicle";
}

BT::NodeStatus ComputePathToGoal::on_result(const carla_nav_msgs::PathPlannerResult &res) {
  goal_result_.path = res.path;
  config().blackboard->set<nav_msgs::Path>("path", goal_result_.path);
  return BT::NodeStatus::SUCCESS;
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
      [](const std::string & name, const BT::NodeConfiguration & config)
      {
        return std::make_unique<ComputePathToGoal>(
            name, "compute_path_to_goal", config);
      };

  factory.registerBuilder<ComputePathToGoal>(
      "ComputePathToGoal", builder);
}
