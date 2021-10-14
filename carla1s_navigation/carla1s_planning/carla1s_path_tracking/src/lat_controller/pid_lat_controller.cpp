
#include "pid_lat_controller.h"

PidLatController::PidLatController() {
  pid_ptr_ = std::make_unique<PIDImpl<double>>(pid_param_fwd_);
}

void PidLatController::Reset(const DirectedPath2dPtr &directed_path_ptr) {
  VehicleController::Reset(directed_path_ptr);
  if(GetDrivingDirection() == DrivingDirection::BACKWARDS){
    pid_ptr_->ResetParam(pid_param_bck_);
  }else{
    pid_ptr_->ResetParam(pid_param_fwd_);
  }
}

NodeState PidLatController::RunStep(const VehicleState &vehicle_state,
                                    const double &target_speed,
                                    const double &dt,
                                    double &steer) {
  UpdateVehicleState(vehicle_state);

  steer = 0.0;
  double lat_error = 0.0;
  auto status = ComputeLatErrors(lat_error);
  if(status == FAILURE){
    return FAILURE;
  }
  steer = pid_ptr_->RunStep(lat_error, dt);
  return SUCCESS;
}

NodeState PidLatController::ComputeLatErrors(double &error) {
  error = 0;
  auto lookahead_dist = 0.05;
  int waypoint_idx = 0;
  auto status = GetPathHandlerPtr()->QueryNearestWaypointIndexWithLookaheadDist(lookahead_dist, waypoint_idx);
  if(status == FAILURE){
    return FAILURE;
  }

  // Use Angle Diff
  SetCurrentWaypointIdx(waypoint_idx);
  error = math::NormalizeAngle(GetCurrentWaypoint().yaw - GetVehiclePose().yaw);
  return SUCCESS;

  // Use cos angle
//    Eigen::Vector2d V, P, next_vec;
//    V << vehicle_pose_ptr->x, vehicle_pose_ptr->y;
//
//    auto index = QueryTargetWaypointIndex(vehicle_pose_ptr, waypoints_ptr);
//    current_waypoint_index_ = index;
//    P << waypoints_ptr->at(index).x, waypoints_ptr->at(index).y;
//
//    next_vec = P - V;
//    next_vec.normalize();
//
//    Eigen::Vector2d heading_vec;
//    heading_vec << std::cos(vehicle_pose_ptr->yaw), std::sin(vehicle_pose_ptr->yaw);
//    heading_vec.normalize();
//
//    double dot_product = std::abs(next_vec.dot(heading_vec));
//    dot_product = math::Clip(dot_product, 0.0, 1.0);
//    double cross_product = heading_vec[0] * next_vec[1] - heading_vec[1] * next_vec[0];
//    dot_product = std::acos(dot_product) / M_PI;
//    if (cross_product > 0.0) {
//      dot_product *= -1.0;
//    }
//    error = dot_product;
//    return SUCCESS;

}

