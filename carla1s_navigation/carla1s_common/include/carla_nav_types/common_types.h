#ifndef CARLA1S_ROS_CARLA_NAVIGATION_CARLA_NAV_COMMON_INCLUDE_TYPES_COMMON_TYPES_H_
#define CARLA1S_ROS_CARLA_NAVIGATION_CARLA_NAV_COMMON_INCLUDE_TYPES_COMMON_TYPES_H_

#include <memory>
#include <utility>
#include <vector>
#include <Eigen/Eigen>

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

  Pose2d(double x, double y, double z, double yaw) : x(x), y(y), yaw(yaw) {};

  Eigen::Matrix3d ToTransformMatrix() const{
    Eigen::Matrix3d trans_matrix;
    trans_matrix << cos(yaw), -sin(yaw), x,
                    sin(yaw), cos(yaw), y,
                    0,0,1;
    return trans_matrix;
  };

  Eigen::Vector3d ToPointVec() const{
    Eigen::Vector3d vec;
    vec << x, y, 1;
    return vec;
  };

  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;

};

using Pose2dPtr = Pose2d::Ptr;
using Path2d = std::vector<Pose2d>;
using Path2dPtr = std::shared_ptr<Path2d>;

enum class DrivingDirection : int {
  FORWARD = 0,
  BACKWARDS = 1
};

struct DirectedPath2d{
  using Ptr = std::shared_ptr<DirectedPath2d>;
  DirectedPath2d(Path2dPtr path_ptr, DrivingDirection driving_direction)
      : path_ptr(std::move(path_ptr)), driving_direction(driving_direction) {}
  Path2dPtr path_ptr;
  DrivingDirection driving_direction;
  uint8_t reverse() const{
    if(driving_direction == DrivingDirection::BACKWARDS){
      return 1;
    } else{
      return 0;
    }
  };
};


using DirectedPath2dPtr = DirectedPath2d::Ptr;

struct VehicleState{
  Pose2d vehicle_pose;
  double vehicle_speed = 0.0;
  DrivingDirection direction = DrivingDirection::FORWARD;
};

#endif // CARLA1S_ROS_CARLA_NAVIGATION_CARLA_NAV_COMMON_INCLUDE_TYPES_COMMON_TYPES_H_