//
// Created by kevinlad on 2021/6/7.
//

#ifndef SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_PATH_TRACKING_SRC_IMPL_IMPL_H_
#define SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_PATH_TRACKING_SRC_IMPL_IMPL_H_

#include "path_tracking_base.h"
#include <cmath>
#include <eigen3/Eigen/Core>

// PurePursuit Implementation
// Ref: http://acl.mit.edu/papers/KuwataGNC08.pdf

class PurePursuit : PathTrackingBase{
 public:

  int Initialize(float wheelbase, float look_ahead_dist_fwd, float anchor_dist_fwd);

  int ComputeAckermannCmd(const Pose2d &vehicle_pose, AckermannCmd &ackermann_cmd) override;

  bool IsGoalReached() override;

  int SetPlan(const Path2d &path) override;

//  void UpdateVehiclePose(const Pose2d &pose) override;

 private:
  float CalculateSteering(const Pose2d &vehicle_pose);

  Pose2d ToVehicleFrame(const Pose2d &point_in_map, const Pose2d &vehicle_pose_in_map);

  inline bool IsForwardWaypoint(const Pose2d &waypoint_pose, const Pose2d &vehicle_pose);

  inline bool IsWaypointAwayFromLookAheadDist(const Pose2d &waypoint_pose, const Pose2d &vehicle_pose);

 private:
  float wheel_base = 0; // Wheelbase (L, distance between front and back wheel)
  float L_fw = 0; // Forward look-ahead distance (L_fw)
  float l_anchor_fw = 0; // Forward anchor distance (L_fw)
  float l_rv = 0; // Reverse look-ahead distance (L_rv)
  float l_anchor_rv = 0; // Reverse anchor distance (l_rv)
  float base_angle = 0.0;
  float steering_gain = 2.0;
  float speed_increment = 2.0;
  float max_speed = 15.0;
  float goal_radius = 1.0;
  bool use_seg = false;


  Path2d path_;
  PathSeg path_seg_;
  Pose2d goal_;
  Pose2d vehicle_pose_;
  bool found_forward_point_;

  float speed_;
  float steering_;

};

#endif //SRC_CARLA_ROS_BRIDGE_CARLA_NAVIGATION_CARLA_PATH_TRACKING_SRC_IMPL_IMPL_H_
