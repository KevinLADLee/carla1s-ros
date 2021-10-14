
#ifndef CARLA1S_ROS_CARLA_NAVIGATION_CARLA_PATH_TRACKING_SRC_VEHICLE_CONTROLLER_BASE_H_
#define CARLA1S_ROS_CARLA_NAVIGATION_CARLA_PATH_TRACKING_SRC_VEHICLE_CONTROLLER_BASE_H_

#include <vector>
#include <memory>

#include <carla_nav_types/common_types.h>
#include <carla_nav_types/conversions.h>
#include <carla_nav_math/math_utils.h>

#include <Eigen/Eigen>
#include "path_handler.h"

class VehicleController{
public:
  VehicleController() {
    path_handler_ptr_ = std::make_shared<PathHandler>();
  }

  virtual void Reset(const DirectedPath2dPtr &directed_path_ptr) {
    directed_path_ptr_ = directed_path_ptr;
    path_handler_ptr_->SetDirectedPathPtr(directed_path_ptr);
  };

  virtual NodeState RunStep(const VehicleState &vehicle_state,
                    const double &target_speed,
                    const double &dt,
                    double &control_output) = 0;

  Pose2d GetCurrentWaypoint() const {
    return directed_path_ptr_->path_ptr->at(current_waypoint_idx_);
  }

 protected:
  virtual NodeState UpdateVehicleState(const VehicleState &vehicle_state){
    vehicle_state_ = vehicle_state;
    path_handler_ptr_->Update(vehicle_state);
    return SUCCESS;
  };

  virtual const DrivingDirection &GetDrivingDirection() const{
    return directed_path_ptr_->driving_direction;
  };

 protected:
  const VehicleState &GetVehicleState() const {
    return vehicle_state_;
  }

  const Pose2d &GetVehiclePose() const {
    return vehicle_state_.vehicle_pose;
  }

  const std::shared_ptr<PathHandler> &GetPathHandlerPtr() const {
    return path_handler_ptr_;
  }

  void SetCurrentWaypointIdx(int current_waypoint_idx) {
    current_waypoint_idx_ = current_waypoint_idx;
  }

 protected:
  std::shared_ptr<PathHandler> path_handler_ptr_;
  VehicleState vehicle_state_;
  DirectedPath2dPtr directed_path_ptr_;
  int current_waypoint_idx_ = 0;
};


#endif //CARLA1S_ROS_CARLA_NAVIGATION_CARLA_PATH_TRACKING_SRC_VEHICLE_CONTROLLER_BASE_H_
