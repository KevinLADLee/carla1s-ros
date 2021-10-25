
#include "stanley.h"

void Stanley::Reset(const DirectedPath2dPtr &directed_path_ptr) {
  VehicleController::Reset(directed_path_ptr);
}

NodeState Stanley::RunStep(const VehicleState &vehicle_state,
                           const double &target_speed,
                           const double &dt,
                           double &control_output) {
  UpdateVehicleState(vehicle_state);
  control_output = ComputeSteer();
  return NodeState::SUCCESS;
}

double Stanley::ComputeSteer() {
  auto v_pose = GetVehiclePose();
  auto v_speed = GetVehicleState().vehicle_speed;

  auto front_axle_pose = ComputeFrontAxlePose(v_pose);

  auto wp_idx = GetPathHandlerPtr()->QueryNearestWaypointIndex(front_axle_pose);
  SetCurrentWaypointIdx(wp_idx);
  auto waypoint = GetCurrentWaypoint();

  // Project RMS error onto front axle vector
  Eigen::Vector2d d_vec;
  d_vec << front_axle_pose.x - waypoint.x,
          front_axle_pose.y - waypoint.y;

  Eigen::Vector2d f_vec;
  f_vec << -std::cos(v_pose.yaw + M_PI / 2.0),
           -std::sin(v_pose.yaw + M_PI / 2.0);

  auto error_front_axle = d_vec.dot(f_vec);

  using namespace carla1s;
  auto theta_e = math::NormalizeAngle(waypoint.yaw - v_pose.yaw);
  auto theta_d = std::atan2(k_ * error_front_axle, v_speed / 3.6);
  auto delta = theta_e + theta_d;

  auto steer = math::Clip(delta, -max_steer_angle_, max_steer_angle_);
//  std::cout << "delta: " << delta << " steer: " << -steer * 2.0 / M_PI << std::endl;
  return -steer * 2.0 / M_PI;
}

Pose2d Stanley::ComputeFrontAxlePose(const Pose2d &vehicle_pose) {
  auto fx = vehicle_pose.x + wheel_base_ * std::cos(vehicle_pose.yaw);
  auto fy = vehicle_pose.y + wheel_base_ * std::sin(vehicle_pose.yaw);
  return Pose2d(fx, fy, 0);
}
