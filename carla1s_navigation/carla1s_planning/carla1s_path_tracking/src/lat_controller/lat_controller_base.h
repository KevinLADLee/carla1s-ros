
#ifndef CARLA1S_ROS_CARLA_NAVIGATION_CARLA_PATH_TRACKING_SRC_LAT_CONTROLLER_LAT_CONTROLLER_BASE_H_
#define CARLA1S_ROS_CARLA_NAVIGATION_CARLA_PATH_TRACKING_SRC_LAT_CONTROLLER_LAT_CONTROLLER_BASE_H_

#include "common/vehicle_controller_base.h"

class LatController : public VehicleController{
 public:
  double RunStep(const Pose2dPtr &vehicle_pose_ptr,
                 const Path2dPtr &waypoints_ptr,
                 const double &vehicle_speed,
                 const double &dt) override {
    return 0;
  }

  int QueryTargetWaypointIndex(const Pose2dPtr &vehicle_pose_ptr,
                               const Path2dPtr &waypoints_ptr) override {
    auto index = QueryNearestWaypointIndex(vehicle_pose_ptr, waypoints_ptr);
    return index;
  }

  virtual double ComputeLatErrors(const Pose2dPtr &vehicle_pose_ptr,
                                  const Path2dPtr &waypoints_ptr){
      return 0;
  };

  void SetMaxSteerAngle(double max_steer_angle){
    max_steer_angle_ = max_steer_angle;
  };

  double GetMaxSteerAngle() const{
    return max_steer_angle_;
  };

 protected:

  double max_steer_angle_ = 1.0;
};

#endif //CARLA1S_ROS_CARLA_NAVIGATION_CARLA_PATH_TRACKING_SRC_LAT_CONTROLLER_LAT_CONTROLLER_BASE_H_
