//
// Created by kevinlad on 2021/5/26.
//

#ifndef SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_PATH_TRACKING_SRC_PLANNER_COMMON_H_
#define SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_PATH_TRACKING_SRC_PLANNER_COMMON_H_

#include <vector>

enum NodeState{
  IDLE,
  RUNNING,
  PAUSE,
  SUCCESS,
  FAILURE
};

struct Pose2d{
  Pose2d() = default;
  Pose2d(double x, double y, double yaw) : x(x), y(y), yaw(yaw) {};
  double x = 0;
  double y = 0;
  double yaw = 0;
};

struct Path2d{
  std::vector<Pose2d> poses;
};

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


#endif //SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_PATH_TRACKING_SRC_PLANNER_COMMON_H_
