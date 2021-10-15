
#ifndef CARLA1S_ROS_CARLA_NAVIGATION_CARLA_PATH_TRACKING_SRC_LON_CONTROLLER_PID_LON_CONTROLLER
#define CARLA1S_ROS_CARLA_NAVIGATION_CARLA_PATH_TRACKING_SRC_LON_CONTROLLER_PID_LON_CONTROLLER

#include "pid_lon_controller.h"

using namespace std;

PidLonController::PidLonController()
{
  station_controller_ = std::make_unique<PIDImpl<double>>(station_pid_param_fwd_);
  speed_controller_ = std::make_unique<PIDImpl<double>>(speed_pid_param_fwd_);
}

void PidLonController::Reset(const DirectedPath2dPtr &directed_path_ptr) {
  VehicleController::Reset(directed_path_ptr);
  if(GetDrivingDirection() == DrivingDirection::BACKWARDS){
    station_controller_->ResetParam(station_pid_param_bck_);
    speed_controller_->ResetParam(speed_pid_param_bck_);
  }else{
    station_controller_->ResetParam(station_pid_param_fwd_);
    speed_controller_->ResetParam(speed_pid_param_fwd_);
  }
}

NodeState PidLonController::RunStep(const VehicleState &vehicle_state,
                                    const double &target_speed,
                                    const double &dt,
                                    double &throttle) {

  VehicleController::UpdateVehicleState(vehicle_state);

  double station_error = 0.0;
  auto status = ComputeLonErrors(station_error);
  if(status == FAILURE){
    ROS_ERROR("PathTracking: ComputeLonError Failed!");
    throttle = 0.0;
    return FAILURE;
  }

  if(target_speed >= 0) {
    station_controller_->SetMax(target_speed);
  }

  auto speed_offset = station_controller_->RunStep(station_error, dt);
  throttle = speed_controller_->RunStep(speed_offset, GetVehicleState().vehicle_speed, dt);
  return SUCCESS;
}



NodeState PidLonController::ComputeLonErrors(double &error){
  auto lookahead_dist = GetLookaheadDist();
  int idx = 0;
  auto status = GetPathHandlerPtr()->QueryNearestWaypointIndexWithLookaheadDist(lookahead_dist, idx);
  if(status == NodeState::FAILURE){
    ROS_ERROR("PathTracking: Query Target Waypoint Failed!");
    return status;
  }
  SetCurrentWaypointIdx(idx);
  auto waypoint = GetCurrentWaypoint();

  Eigen::Matrix3d waypoint_trans;
  auto theta = waypoint.yaw;
  waypoint_trans << cos(theta), -sin(theta), waypoint.x,
      sin(theta), cos(theta), waypoint.y,
      0,0,1;

  auto waypoint_trans_inv = waypoint_trans.inverse();

  Eigen::Vector3d vehicle_point_vec(GetVehicleState().vehicle_pose.x, GetVehicleState().vehicle_pose.y, 1);
  auto vehicle_point_in_waypoint_frame = waypoint_trans_inv * vehicle_point_vec;

  error = -vehicle_point_in_waypoint_frame.x();
  if(error > 0){
    return NodeState::RUNNING;
  }else{
    return NodeState::FAILURE;
  }
}

double PidLonController::GetLookaheadDist() const {
  if(GetDrivingDirection() == DrivingDirection::FORWARD){
    return max_fwd_lookahead_dist;
  }else{
    return max_bck_lookahead_dist;
  }
}

double PidLonController::GetSpeedError() {
  return speed_controller_->GetPreError();
}

double PidLonController::GetStationError() {
  return station_controller_->GetPreError();
}

#endif // #ifndef CARLA1S_ROS_CARLA_NAVIGATION_CARLA_PATH_TRACKING_SRC_LON_CONTROLLER_PID_LON_CONTROLLER