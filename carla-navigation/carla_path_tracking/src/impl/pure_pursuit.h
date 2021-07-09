//
// Created by kevinlad on 2021/6/7.
//

#ifndef CARLA1S_ROS_CARLA_NAVIGATION_CARLA_PATH_TRACKING_SRC_IMPL_IMPL_H_
#define CARLA1S_ROS_CARLA_NAVIGATION_CARLA_PATH_TRACKING_SRC_IMPL_IMPL_H_

#include "path_tracking_base.h"
#include <cmath>
#include <Eigen/Dense>
#include <iostream>

// PurePursuit Implementation
// Ref: http://acl.mit.edu/papers/KuwataGNC08.pdf

class PurePursuit : public LateralController{
 public:

  int Initialize(float wheelbase, float goal_radius,float look_ahead_dist_fwd = 2.5, float anchor_dist_fwd = 1.5);

  double RunStep(const Pose2dPtr &vehicle_pose,
                 const Path2dPtr &waypoints) override;

  Pose2d GetCurrentTrackPoint();

 private:
  int FindValidWaypoint(const Pose2dPtr &vehicle_pose_ptr, Pose2dPtr &valid_waypoint_ptr);

  double ComputeSteering(const Pose2dPtr &vehicle_pose, const Pose2dPtr &valid_waypoint);

  Pose2d ToVehicleFrame(const Pose2d &point_in_map, const Pose2d &vehicle_pose_in_map);

  inline bool IsValidWaypoint(const Pose2d &waypoint_pose, const Pose2d &vehicle_pose);

 private:
  float wheel_base = 0; // Wheelbase (L, distance between front and back wheel)
  float L_fw = 3.0; // Forward look-ahead distance (L_fw)
  float l_anchor_fw = 1.5; // Forward anchor distance (L_fw)
  float L_rv = 0.5; // Reverse look-ahead distance (L_rv)
  float l_anchor_rv = 0.0; // Reverse anchor distance (l_rv)
  bool found_valid_waypoint_ = false;

  Pose2dPtr valid_waypoint_ptr_;
  int current_waypoint_index_ = 0;
  Pose2d current_waypoint;
  double max_steering_angle_ = 1.0;

};

#endif //CARLA1S_ROS_CARLA_NAVIGATION_CARLA_PATH_TRACKING_SRC_IMPL_IMPL_H_
