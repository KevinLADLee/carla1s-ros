
#include "compute_path_to_goal.h"

ComputePathToGoal::ComputePathToGoal(const std::string &name, const std::string &action_client_name, const BT::NodeConfiguration &conf)
    : RosActionNode(name, action_client_name, conf) {
  first_goal_received_ = false;
}

void ComputePathToGoal::on_tick() {
  BT::Optional<std::string> path_planner_id = getInput<std::string>("path_planner_id");
  if (!path_planner_id)
  {
    goal_.planner_id = "waypoint";
    throw BT::RuntimeError("missing required input [planner_id]: ", path_planner_id.error());
  }
  goal_.planner_id = path_planner_id.value();

  BT::Optional<geometry_msgs::PoseStamped> goal_pose = getInput<geometry_msgs::PoseStamped>("goal");
  if (!goal_pose)
  {
    setStatus(BT::NodeStatus::FAILURE);
    throw BT::RuntimeError("missing required input [goal]: ", goal_pose.error());
  }
  goal_.goal = goal_pose.value();
  goal_.role_name = "ego_vehicle";
}

BT::NodeStatus ComputePathToGoal::on_result(const carla_nav_msgs::PathPlannerResult &res) {
  goal_result_.path = res.path;
  config().blackboard->set<carla_nav_msgs::Path>("path", goal_result_.path);
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
