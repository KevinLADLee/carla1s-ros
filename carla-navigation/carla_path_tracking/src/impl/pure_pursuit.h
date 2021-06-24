//
// Created by kevinlad on 2021/6/7.
//

#ifndef SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_PATH_TRACKING_SRC_IMPL_IMPL_H_
#define SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_PATH_TRACKING_SRC_IMPL_IMPL_H_

#include "path_tracking_base.h"
#include <cmath>
#include <Eigen/Dense>
#include <iostream>

// PurePursuit Implementation
// Ref: http://acl.mit.edu/papers/KuwataGNC08.pdf

class PurePursuit : public PathTrackingBase{
 public:

  int Initialize(float wheelbase, float look_ahead_dist_fwd = 2.5, float anchor_dist_fwd = 1.5);

  int ComputeAckermannCmd(const Pose2d &vehicle_pose, AckermannCmd &ackermann_cmd) override;

  bool IsGoalReached() override;

  int SetPlan(const Path2d &path, const DrivingDirection &path_direction) override;

  Pose2d GetCurrentTrackPoint();

 private:
  int CalculateSteering(const Pose2d &vehicle_pose, float &steering);

  Pose2d ToVehicleFrame(const Pose2d &point_in_map, const Pose2d &vehicle_pose_in_map);

  inline bool IsValidWaypoint(const Pose2d &waypoint_pose, const Pose2d &vehicle_pose);

  float DistToGoal(const Pose2d& pose);

  void GetTargetSpeedAndAcc(float &acc, float &speed);

 private:
  float wheel_base = 0; // Wheelbase (L, distance between front and back wheel)
  float L_fw = 2.5; // Forward look-ahead distance (L_fw)
  float l_anchor_fw = 1.5; // Forward anchor distance (L_fw)
  float L_rv = 1.5; // Reverse look-ahead distance (L_rv)
  float l_anchor_rv = 0.1; // Reverse anchor distance (l_rv)
  float base_angle = 0.0;
  float steering_gain = 2.0;
  float max_forward_speed_ = 5.0; // ~20km/h
  float max_forward_acc_ = 5.1; // ~20km/h
  float max_backwards_speed_ = 1.0; // ~20km/h
  float max_backwards_acc_ = 1.1; // ~20km/h

  float goal_radius = 1.0;
  float safe_dist = 5.0;

  Path2d path_;
  DrivingDirection path_direction_ = DrivingDirection::FORWARD;
  Pose2d goal_;
  Pose2d vehicle_pose_;
  bool found_valid_waypoint_ = false;

  std::vector<Pose2d>::iterator current_waypoint_it_;
  int current_waypoint_index_ = 0;

  float speed_ = 0.0;
  float acc_ = 0.0;
  float steering_ = 0.0;

};

#endif //SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_PATH_TRACKING_SRC_IMPL_IMPL_H_
