#ifndef CARLA1S_ROS_CARLA_NAVIGATION_CARLA_NAV_COMMON_INCLUDE_TYPES_CONVERSIONS_H_
#define CARLA1S_ROS_CARLA_NAVIGATION_CARLA_NAV_COMMON_INCLUDE_TYPES_CONVERSIONS_H_

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <tf2/utils.h>
#include <carla1s_msgs/PathArray.h>

#include "common_types.h"

static Path2d RosPathToPath2d(const nav_msgs::Path &ros_path) {
  Path2d path2d;
  path2d.reserve(ros_path.poses.size());
  for(const auto& pose_ros_it : ros_path.poses){
    auto yaw = tf2::getYaw(pose_ros_it.pose.orientation);
    path2d.emplace_back(pose_ros_it.pose.position.x, pose_ros_it.pose.position.y, yaw);
  }
  return path2d;
}

static geometry_msgs::Pose Pose2dToRosMsg(const Pose2d &pose2d) {
  geometry_msgs::Pose ros_pose;
  ros_pose.position.x = pose2d.x;
  ros_pose.position.y = pose2d.y;
  tf2::Quaternion q;
  q.setRPY(0,0,pose2d.yaw);
  tf2::convert(q, ros_pose.orientation);
  return ros_pose;
}

static Pose2d RosPoseStampedToPose2d(const geometry_msgs::PoseStamped &pose) {
  Pose2d pose2d;
  pose2d.x = pose.pose.position.x;
  pose2d.y = pose.pose.position.y;
  pose2d.yaw = tf2::getYaw(pose.pose.orientation);
  return pose2d;
}

static DrivingDirection DrivingDirectionMsgToDirection(const int8_t &dire_msg) {
  if(dire_msg == carla1s_msgs::PathArray::FORWARD) {
    return DrivingDirection::FORWARD;
  } else{
    return DrivingDirection::BACKWARDS;
  }
}


#endif // CARLA1S_ROS_CARLA_NAVIGATION_CARLA_NAV_COMMON_INCLUDE_TYPES_CONVERSIONS_H_