//
// Created by kevinlad on 2021/5/26.
//

#ifndef SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_PATH_TRACKING_SRC_PATH_TRACKING_H_
#define SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_PATH_TRACKING_SRC_PATH_TRACKING_H_

#include <memory>
#include <mutex>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

#include <carla_nav_msgs/TrackingPathAction.h>
#include <carla_msgs/CarlaEgoVehicleInfo.h>

#include "planner_common.h"

class PathTracking {
 public:
  PathTracking();

  void OdomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg);

  void VehicleInfoCallback(const carla_msgs::CarlaEgoVehicleInfoConstPtr &vehicle_info_msg);

  void PathCallback(const nav_msgs::Path::ConstPtr &path_msg);

 private:

  NodeState getNodeState();

  void setNodeState(const NodeState & node_state);

  bool GoalReached();

 private:
  ros::NodeHandle nh_;

  ros::Subscriber odom_sub_;

};

#endif //SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_PATH_TRACKING_SRC_PATH_TRACKING_H_
