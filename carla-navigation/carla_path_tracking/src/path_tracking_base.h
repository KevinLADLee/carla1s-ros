//
// Created by kevinlad on 2021/6/7.
//

#ifndef SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_PATH_TRACKING_SRC_PATH_TRACKING_BASE_H_
#define SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_PATH_TRACKING_SRC_PATH_TRACKING_BASE_H_

#include <queue>
#include "planner_common.h"

class PathTrackingBase{
 public:
//  virtual int Initialize() = 0;

  virtual int ComputeAckermannCmd(const Pose2d &vehicle_pose, AckermannCmd &ackermann_cmd) = 0;

  virtual bool IsGoalReached() = 0;

  virtual int SetPlan(const Path2d &path, const DrivingDirection &path_direction) = 0;

//  virtual int SetPlan(const Path &path) {
//
//  };

//  virtual void UpdateVehiclePose(const Pose2d &pose) = 0;

};

#endif //SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_PATH_TRACKING_SRC_PATH_TRACKING_BASE_H_
