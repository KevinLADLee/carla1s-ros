
#include "pid_lat_controller.h"

PidLatController::PidLatController() {
  pid_ptr_ = std::make_unique<PIDImpl<double>>(pid_param_fwd_);
}

double PidLatController::ComputeLatErrors(const Pose2dPtr &vehicle_pose_ptr,
                                          const Path2dPtr &waypoints_ptr) {

//  if(GetDrivingDirection() == DrivingDirection::FORWARD ) {
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
//    return dot_product;
//  }

    auto vehicle_yaw = vehicle_pose_ptr->yaw;
    auto wp_index = QueryNearestWaypointIndex(vehicle_pose_ptr, waypoints_ptr);
    current_waypoint_index_ = wp_index;
    auto wp_yaw = waypoints_ptr->at(wp_index).yaw;
    auto error = math::NormalizeAngle(wp_yaw - vehicle_yaw);
//    auto error = wp_yaw - vehicle_yaw;

    if(GetDrivingDirection() == DrivingDirection::FORWARD){
      error *= -1.0;
    }

    return error;
}

double PidLatController::RunStep(const Pose2dPtr &vehicle_pose_ptr,
                                 const Path2dPtr &waypoints_ptr,
                                 const double &vehicle_speed,
                                 const double &dt) {
  if(waypoints_ptr_ != waypoints_ptr){
    waypoints_ptr_ = waypoints_ptr;
    current_waypoint_index_ = 0;
  }

  double lat_error = ComputeLatErrors(vehicle_pose_ptr, waypoints_ptr);
  SetLatestError(lat_error);

  auto steer = pid_ptr_->RunStep(lat_error, dt);
  std::cout << "steer_error: " << lat_error << std::endl;
  return steer;
}

int PidLatController::SetDrivingDirection(const DrivingDirection &driving_direction) {
  if(driving_direction != driving_direction_){
    if(driving_direction == DrivingDirection::FORWARD){
      pid_ptr_ = std::make_unique<PIDImpl<double>>(pid_param_fwd_);
    }else{
      pid_ptr_ = std::make_unique<PIDImpl<double>>(pid_param_bck_);
    }
  }
  VehicleController::SetDrivingDirection(driving_direction);
  return 0;
}
