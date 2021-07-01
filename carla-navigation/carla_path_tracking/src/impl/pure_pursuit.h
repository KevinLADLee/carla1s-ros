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

class PurePursuit : public LateralController{
 public:

  int Initialize(float wheelbase, float goal_radius,float look_ahead_dist_fwd = 2.5, float anchor_dist_fwd = 1.5);

  double RunStep(const Pose2dPtr &vehicle_pose,
                 const Path2dPtr &waypoints = nullptr) override;

  int SetPlan(const Path2d &path, const DrivingDirection &driving_direction) override;

  Pose2d GetCurrentTrackPoint();

  bool IsGoalReached();

 private:
  int CalculateSteering(const Pose2d &vehicle_pose, double &steering);

  Pose2d ToVehicleFrame(const Pose2d &point_in_map, const Pose2d &vehicle_pose_in_map);

  inline bool IsValidWaypoint(const Pose2d &waypoint_pose, const Pose2d &vehicle_pose);

  float DistToGoal(const Pose2d& pose);

 private:
  float wheel_base = 0; // Wheelbase (L, distance between front and back wheel)
  float L_fw = 3.0; // Forward look-ahead distance (L_fw)
  float l_anchor_fw = 1.5; // Forward anchor distance (L_fw)
  float L_rv = 2.0; // Reverse look-ahead distance (L_rv)
  float l_anchor_rv = 1.0; // Reverse anchor distance (l_rv)

  float goal_radius_;

  Path2d path_;
  Pose2d goal_;
  Pose2d vehicle_pose_;
  bool found_valid_waypoint_ = false;

  std::vector<Pose2d>::iterator current_waypoint_it_;
  int current_waypoint_index_ = 0;

  double max_steering_angle_ = 1.0;

};

#endif //SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_PATH_TRACKING_SRC_IMPL_IMPL_H_
