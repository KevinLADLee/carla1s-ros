
#ifndef CARLA1S_ROS_CARLA_NAVIGATION_CARLA_PATH_TRACKING_SRC_IMPL_IMPL_H_
#define CARLA1S_ROS_CARLA_NAVIGATION_CARLA_PATH_TRACKING_SRC_IMPL_IMPL_H_

#include <common/vehicle_controller_base.h>

#include <cmath>
#include <Eigen/Eigen>
#include <iostream>

// PurePursuit Implementation
// Ref: http://acl.mit.edu/papers/KuwataGNC08.pdf

class PurePursuit : public VehicleController{
 public:

  int Initialize(float wheelbase, float goal_radius,float look_ahead_dist_fwd = 2.5, float anchor_dist_fwd = 1.5);

  NodeState RunStep(const VehicleState &vehicle_state,
                    const double &target_speed,
                    const double &dt,
                    double &steer) override;

  void Reset(const DirectedPath2dPtr &directed_path_ptr) override;

 private:

  Pose2d ToVehicleFrame(const Pose2d &point_in_map, const Pose2d &vehicle_pose_in_map);

  double GetLookaheadDist() const;

  double ComputeSteering(const Pose2d &valid_waypoint);

 private:
  float wheel_base = 2.5; // Wheelbase (L, distance between front and back wheel)
  float max_fwd_lookahead_dist = 3.0; // Forward look-ahead distance (L_fw)
  float l_anchor_fw = 1.5; // Forward anchor distance (L_fw)
  float max_bck_lookahead_dist = 2.5; // Reverse look-ahead distance (L_rv)
  float l_anchor_rv = 0.0; // Reverse anchor distance (l_rv)

  double max_steering_angle = 1.0;

};

#endif //CARLA1S_ROS_CARLA_NAVIGATION_CARLA_PATH_TRACKING_SRC_IMPL_IMPL_H_
