#ifndef CARLA1S_ROS_CARLA_NAVIGATION_CARLA_NAV_COMMON_INCLUDE_TYPES_COMMON_TYPES_H_
#define CARLA1S_ROS_CARLA_NAVIGATION_CARLA_NAV_COMMON_INCLUDE_TYPES_COMMON_TYPES_H_

#include <vector>

enum NodeState{
  IDLE = 0,
  RUNNING = 1,
  PAUSE = 2,
  SUCCESS = 3,
  FAILURE = 4
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

#endif // CARLA1S_ROS_CARLA_NAVIGATION_CARLA_NAV_COMMON_INCLUDE_TYPES_COMMON_TYPES_H_