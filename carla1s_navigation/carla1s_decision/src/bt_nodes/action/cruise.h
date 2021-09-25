
#ifndef CARLA1S_ROS_CARLA_NAVIGATION_CARLA_DECISION_SRC_BT_NODES_ACTION_CRUISE_H_
#define CARLA1S_ROS_CARLA_NAVIGATION_CARLA_DECISION_SRC_BT_NODES_ACTION_CRUISE_H_

#include "carla1s_decision_common.h"

#include <carla1s_msgs/PathTrackingAction.h>
#include <carla1s_msgs/PathPlannerAction.h>

 class Cruise : public BT::RosPlanningActionNode<carla1s_msgs::PathPlannerAction, carla1s_msgs::PathTrackingAction> {

  protected:
   void GlobalPlannerFeedbackCallback(const GlobalPlannerActionFeedbackT::ConstPtr &global_planner_feedback) override;
  public:
   Cruise(const std::string &bt_node_name,
          const std::string &global_action_client_name,
          const std::string &local_action_client_name,
          const BT::NodeConfiguration &conf);
   ~Cruise() override = default;
 };

#endif //CARLA1S_ROS_CARLA_NAVIGATION_CARLA_DECISION_SRC_BT_NODES_ACTION_CRUISE_H_
