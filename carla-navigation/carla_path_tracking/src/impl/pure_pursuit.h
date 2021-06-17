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

  int Initialize(float wheelbase, float max_speed = 8.0, float look_ahead_dist_fwd = 2.5, float anchor_dist_fwd = 1.5);

  int ComputeAckermannCmd(const Pose2d &vehicle_pose, AckermannCmd &ackermann_cmd) override;

  bool IsGoalReached() override;

  int SetPlan(const Path2d &path, const PathDirection &path_direction) override;

//  void UpdateVehiclePose(const Pose2d &pose) override;


  Pose2d GetCurrentTrackPoint();

 private:
  float CalculateSteering(const Pose2d &vehicle_pose);

  Pose2d ToVehicleFrame(const Pose2d &point_in_map, const Pose2d &vehicle_pose_in_map);

  inline bool IsForwardWaypoint(const Pose2d &waypoint_pose, const Pose2d &vehicle_pose);

  inline bool IsWaypointAwayFromLookAheadDist(const Pose2d &waypoint_pose, const Pose2d &vehicle_pose);

  float DistToGoal(const Pose2d& pose);

 private:
  float wheel_base = 0; // Wheelbase (L, distance between front and back wheel)
  float L_fw = 2.5; // Forward look-ahead distance (L_fw)
  float l_anchor_fw = 1.5; // Forward anchor distance (L_fw)
  float l_rv = 0; // Reverse look-ahead distance (L_rv)
  float l_anchor_rv = 0.0; // Reverse anchor distance (l_rv)
  float base_angle = 0.0;
  float steering_gain = 2.0;
  float speed_increment = 2.0;
  float max_speed_ = 5.5; // ~20km/h
  float goal_radius = 1.0;
  float safe_dist = 5.0;
  bool use_seg = false;

  Path2d path_;
  PathDirection path_direction_ = PathDirection::FWD;
  Pose2d goal_;
  Pose2d vehicle_pose_;
  bool found_forward_point_ = false;

  std::vector<Pose2d>::iterator current_waypoint_it_;
  int current_waypoint_index_ = 0;

  float speed_ = 0.0;
  float steering_ = 0.0;

};

#endif //SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_PATH_TRACKING_SRC_IMPL_IMPL_H_
