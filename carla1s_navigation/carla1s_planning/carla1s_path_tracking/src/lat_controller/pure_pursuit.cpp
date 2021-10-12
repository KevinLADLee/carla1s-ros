
#include "pure_pursuit.h"

void PurePursuit::Reset(const DirectedPath2dPtr &directed_path_ptr) {
  VehicleController::Reset(directed_path_ptr);
}

NodeState PurePursuit::RunStep(const VehicleState &vehicle_state,
                               const double &target_speed,
                               const double &dt,
                               double &steer) {
  UpdateVehicleState(vehicle_state);
  steer = 0.0;
  int waypoint_idx = 0;
  auto lookahead_dist = GetLookaheadDist();
  auto status = GetPathHandlerPtr()->QueryNearestWaypointIndexWithLookaheadDist(lookahead_dist, waypoint_idx);
  SetCurrentWaypointIdx(waypoint_idx);
  if(status == NodeState::FAILURE){
    return FAILURE;
  }
  steer = ComputeSteering(GetCurrentWaypoint());
  std::cout << steer << std::endl;
  return SUCCESS;
}

double PurePursuit::ComputeSteering(const Pose2d &valid_waypoint){
  double steering = 0.0;
  auto forward_point_in_vehicle_frame = ToVehicleFrame(valid_waypoint, GetVehiclePose());
  if(GetDrivingDirection() == DrivingDirection::FORWARD){
    double eta = std::atan2(forward_point_in_vehicle_frame.y, forward_point_in_vehicle_frame.x);
    steering = std::atan2(wheel_base * std::sin(eta), (GetLookaheadDist() / 2.0f + l_anchor_fw * std::cos(eta)));
  }
  else{
    double eta = M_PI + std::atan2(forward_point_in_vehicle_frame.y, forward_point_in_vehicle_frame.x);
    steering = -std::atan2(wheel_base * std::sin(eta), (GetLookaheadDist() / 2.0f + l_anchor_rv * std::cos(eta)));
  }
  steering = -carla1s::math::Clip(steering, -max_steering_angle, max_steering_angle);
  return steering;
}

int PurePursuit::Initialize(float wheelbase, float goal_radius, float look_ahead_dist_fwd, float anchor_dist_fwd) {
  std::cout << "wheel_base: " << wheelbase << "\n" << std::endl;
  wheel_base = wheelbase;
  max_fwd_lookahead_dist = look_ahead_dist_fwd;
  l_anchor_fw = anchor_dist_fwd;
  return 0;
}

double PurePursuit::GetLookaheadDist() const {
  if(GetDrivingDirection() == DrivingDirection::FORWARD){
    return max_fwd_lookahead_dist;
  }else{
    return max_bck_lookahead_dist;
  }
}

Pose2d PurePursuit::ToVehicleFrame(const Pose2d &point_in_map, const Pose2d &vehicle_pose_in_map) {
  Eigen::Matrix3d vehicle_trans;
  auto theta = vehicle_pose_in_map.yaw;
  using namespace std;
  vehicle_trans << cos(theta), -sin(theta), vehicle_pose_in_map.x,
      sin(theta), cos(theta), vehicle_pose_in_map.y,
      0,0,1;
  auto vehicle_trans_inv = vehicle_trans.inverse();
  Eigen::Vector3d point_vec(point_in_map.x, point_in_map.y, 1);
  auto point_vec_in_vehicle_frame = vehicle_trans_inv * point_vec;
  return {point_vec_in_vehicle_frame[0], point_vec_in_vehicle_frame[1], point_in_map.yaw-vehicle_pose_in_map.yaw};
}
