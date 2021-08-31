
#include "compute_path_to_goal.h"

ComputePathToGoal::ComputePathToGoal(const std::string &name, const std::string &action_client_name, const BT::NodeConfiguration &conf)
    : RosActionNode(name, action_client_name, conf) {
  goal_received_ = false;
}

void ComputePathToGoal::on_tick() {
  BT::Optional<std::string> path_planner_id = getInput<std::string>("path_planner_id");
  if (!path_planner_id)
  {
    goal_.planner_id = "waypoint";
    throw BT::RuntimeError("missing required input [planner_id]: ", path_planner_id.error());
  }
  goal_.planner_id = path_planner_id.value();

  config().blackboard->get<bool>("goal_received", goal_received_);
  if(goal_received_) {
    BT::Optional<geometry_msgs::PoseStamped> goal_pose = getInput<geometry_msgs::PoseStamped>("goal");
    if (!goal_pose.has_value()) {
      setStatus(BT::NodeStatus::FAILURE);
      ROS_WARN("ComputePath: missing required input [goal]");
      return;
    } else {
      goal_.goal = goal_pose.value();
      goal_.role_name = "ego_vehicle";
    }
  }else{
    // Goal has not been updated, return failed;
    setStatus(BT::NodeStatus::FAILURE);
    return;
  }
}

BT::NodeStatus ComputePathToGoal::on_result(const carla_nav_msgs::PathPlannerResult &res) {
  goal_result_.path = res.path;
  setOutput("path", goal_result_.path);
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
