//
// Created by kevinlad on 2021/6/10.
//

#ifndef SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_DECISION_SRC_BT_NODES_ACTION_CRUISE_H_
#define SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_DECISION_SRC_BT_NODES_ACTION_CRUISE_H_

#include "common.h"

#include <carla_nav_msgs/PathTrackingAction.h>
#include <carla_nav_msgs/GlobalPlannerAction.h>

 class Cruise : public BT::RosPlanningActionNode<carla_nav_msgs::GlobalPlannerAction, carla_nav_msgs::PathTrackingAction> {

  protected:
   void GlobalPlannerFeedbackCallback(const GlobalPlannerActionFeedbackT::ConstPtr &global_planner_feedback) override;
  public:
   Cruise(const std::string &bt_node_name,
          const std::string &global_action_client_name,
          const std::string &local_action_client_name,
          const BT::NodeConfiguration &conf);
   ~Cruise() override = default;
 };

#endif //SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_DECISION_SRC_BT_NODES_ACTION_CRUISE_H_
