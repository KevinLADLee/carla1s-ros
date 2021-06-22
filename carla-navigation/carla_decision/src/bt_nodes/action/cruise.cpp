//
// Created by kevinlad on 2021/6/10.
//

#include "cruise.h"

Cruise::Cruise(const std::string &bt_node_name,
               const std::string &global_action_client_name,
               const std::string &local_action_client_name,
               const BT::NodeConfiguration &conf) : RosPlanningActionNode(bt_node_name,
                                                                          global_action_client_name,
                                                                          local_action_client_name,
                                                                          conf) {}

void Cruise::GlobalPlannerFeedbackCallback(const GlobalPlannerActionFeedbackT::ConstPtr &global_planner_feedback) {
  if(!global_planner_feedback->path.poses.empty())
  {
//    local_goal_.path = global_planner_feedback->path;
    local_planner_action_client_->sendGoal(local_goal_);
  }
}

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder =
      [](const std::string & bt_node_name, const BT::NodeConfiguration & config)
      {
        return std::make_unique<Cruise>(
            bt_node_name, "global_planner","path_tracking", config);
      };

  factory.registerBuilder<Cruise>(
      "Cruise", builder);
}