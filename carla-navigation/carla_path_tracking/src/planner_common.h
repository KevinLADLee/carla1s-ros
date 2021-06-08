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
  Pose2d(float x, float y, float yaw) : x(x), y(y), yaw(yaw) {};
  float x = 0;
  float y = 0;
  float yaw = 0;
};

struct Path2d{
  std::vector<Pose2d> poses;
};

enum class PathDirection : int{
  FWD = 1,
  BCK = -1
};

struct PathWithDirection{
  std::vector<Pose2d> poses;
  PathDirection directions;
};

struct PathSeg{
  std::vector<PathWithDirection> paths;
};

struct AckermannCmd{
  float speed = 0;
  float steering_angle = 0;
};


#endif //SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_PATH_TRACKING_SRC_PLANNER_COMMON_H_
