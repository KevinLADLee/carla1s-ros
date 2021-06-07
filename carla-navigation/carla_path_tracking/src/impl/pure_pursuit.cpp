//
// Created by kevinlad on 2021/6/7.
//

#include "pure_pursuit.h"

int PurePursuit::ComputeAckermannCmd(const Pose2d &vehicle_pose,
                                     AckermannCmd &ackermann_cmd) {

  CalculateEta(vehicle_pose);

}

bool PurePursuit::IsGoalReached() {
  return false;
}

int PurePursuit::SetPlan(const Path2d &path) {
  path_ = path;
  return 0;
}

float PurePursuit::CalculateEta(const Pose2d &vehicle_pose) {

  if(!use_seg) {

    Pose2d look_ahead_point;
    found_forward_point_ = false;

    // make sure vehicle_pose is in map frame
    if (IsGoalReached()) {
      for (int i = 0; i < path_.path.size(); i++) {
        auto waypoint_in_map = path_.path.at(i);

      }
    }

    return std::atan2(look_ahead_point.y, look_ahead_point.x);
  }
}

bool PurePursuit::IsForwardWaypoint(const Pose2d &waypoint_pose, const Pose2d &vehicle_pose) {

  return false;
}

int PurePursuit::Initialize(float wheelbase, float look_ahead_dist_fwd, float anchor_dist_fwd) {
  wheel_base = wheelbase;
  l_fw = look_ahead_dist_fwd;
  l_anchor_fw = anchor_dist_fwd;
  return 0;
}
