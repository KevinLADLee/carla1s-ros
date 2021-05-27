//
// Created by kevinlad on 2021/5/26.
//

#include "path_tracking.h"

PathTracking::PathTracking() : nh_({ros::NodeHandle()}){
  std::make_unique<ActionServerT>(nh_,
                                  "path_tracking_action",
                                  boost::bind(&PathTracking::ActionExecuteCallback,this,_1),
                                  false);
}

void PathTracking::ActionExecuteCallback(const ActionGoalT::ConstPtr &action_goal_msg) {

}

void PathTracking::OdomCallback(const nav_msgs::Odometry_<std::allocator<void>>::ConstPtr &odom_msg) {
  std::lock_guard<std::mutex> lock_guard(odom_mutex_);
}

void PathTracking::VehicleInfoCallback(const carla_msgs::CarlaEgoVehicleInfoConstPtr &vehicle_info_msg) {

}

void PathTracking::PathCallback(const nav_msgs::Path_<std::allocator<void>>::ConstPtr &path_msg) {

}

NodeState PathTracking::GetNodeState() {
  return SUCCESS;
}


void PathTracking::SetNodeState(const NodeState &node_state) {

}


bool PathTracking::GoalReached() {
  return false;
}
