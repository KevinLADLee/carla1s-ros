//
// Created by kevinlad on 2021/5/26.
//

#ifndef CARLA1S_ROS_CARLA_NAVIGATION_CARLA_PATH_TRACKING_SRC_PLANNER_COMMON_H_
#define CARLA1S_ROS_CARLA_NAVIGATION_CARLA_PATH_TRACKING_SRC_PLANNER_COMMON_H_

#include <vector>
#include <memory>

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/utils.h>

enum NodeState{
  IDLE,
  RUNNING,
  PAUSE,
  SUCCESS,
  FAILURE
};

struct Pose2d{
  using Ptr = std::shared_ptr<Pose2d>;
  Pose2d() = default;
  Pose2d(double x, double y, double yaw) : x(x), y(y), yaw(yaw) {};
  double x = 0;
  double y = 0;
  double yaw = 0;
};

using Pose2dPtr = Pose2d::Ptr;
using Path2d = std::vector<Pose2d>;
using Path2dPtr = std::shared_ptr<Path2d>;

enum class DrivingDirection : int {
  FORWARD = 0,
  BACKWARDS = 1
};

struct PathSegment{
  std::vector<Pose2d> poses;
  DrivingDirection driving_direction;
};

struct Path{
  std::vector<PathSegment> path_segment;
};

struct AckermannCmd{
  float speed = 0.0;
  float acceleration = 0.0;
  float steering_angle = 0.0;
};

template <typename T>
T clip(const T& n, const T& lower, const T& upper) {
  return std::max(lower, std::min(n, upper));
}

static Path2d RosPathToPath2d(const nav_msgs::Path &ros_path) {
  Path2d path2d;
  path2d.reserve(ros_path.poses.size());
  for(const auto& pose_ros_it : ros_path.poses){
    auto yaw = tf2::getYaw(pose_ros_it.pose.orientation);
    path2d.emplace_back(pose_ros_it.pose.position.x, pose_ros_it.pose.position.y, yaw);
  }
  return path2d;
}






#endif //CARLA1S_ROS_CARLA_NAVIGATION_CARLA_PATH_TRACKING_SRC_PLANNER_COMMON_H_
