//
// Created by kevinlad on 2021/6/10.
//

#ifndef SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_DECISION_SRC_BT_NODE_BASE_BT_PLANNER_NODE_H_
#define SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_DECISION_SRC_BT_NODE_BASE_BT_PLANNER_NODE_H_

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include "common.h"

namespace BT
{

/** Helper Node to call an actionlib::SimpleActionClient<>
 * inside a BT::ActionNode.
 *
 * Note that the user must implement the methods:
 *
 *  - sendGoal
 *  - onResult
 *  - onFailedRequest
 *  - halt (optionally)
 *
 */
template<class GlobalPlannerActionT, class LocalPlannerActionT>
class RosPlanningActionNode : public BT::ActionNodeBase{
 public:
  using GlobalPlannerActionClientT = actionlib::SimpleActionClient<GlobalPlannerActionT>;
  using LocalPlannerActionClientT = actionlib::SimpleActionClient<LocalPlannerActionT>;

 protected:
  RosPlanningActionNode(const std::string& name,
                        const std::string & global_action_client_name,
                        const std::string & local_action_client_name,
                        const BT::NodeConfiguration & conf): BT::ActionNodeBase(name, conf){
    nh_ = config().blackboard->template get<ros::NodeHandlePtr>("node_handler");
    action_client_ = std::make_shared<ActionClientType>( *nh_, action_client_name_, true );

    std::cout << "[BT ROS Action Node]: " << "Waiting action server (" << action_client_name_ << ")..." << std::endl;
    action_client_->waitForServer();
    std::cout << "[BT ROS Action Node]: Action Server (" << action_client_name_ << ") connected success!" << std::endl;
  }

 private:
  ros::NodeHandlePtr nh_;
  GlobalPlannerActionClientT global_planner_action_client_;
  LocalPlannerActionClientT local_planner_action_client_;


};

#endif //SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_DECISION_SRC_BT_NODE_BASE_BT_PLANNER_NODE_H_
