
#ifndef CARLA1S_ROS_CARLA1S_NAVIGATION_CARLA1S_PLANNING_CARLA1S_PATH_TRACKING_SRC_LAT_CONTROLLER_STANLEY_H_
#define CARLA1S_ROS_CARLA1S_NAVIGATION_CARLA1S_PLANNING_CARLA1S_PATH_TRACKING_SRC_LAT_CONTROLLER_STANLEY_H_

#include "common/vehicle_controller_base.h"
#include "carla_nav_math/math_utils.h"

class Stanley : public VehicleController{
 public:
  Stanley(double k = 1.0) : k_(k){};

  NodeState RunStep(const VehicleState &vehicle_state,
                    const double &target_speed,
                    const double &dt,
                    double &control_output) override;
  void Reset(const DirectedPath2dPtr &directed_path_ptr) override;

 private:
  inline double ComputeSteer();

  Pose2d ComputeFrontAxlePose(const Pose2d &vehicle_pose);

 private:
  double k_ = 0.5; // gain factor
  double wheel_base_ = 3.0;
  double max_steer_angle_ = 70.0 / 180.0 * M_PI;
  double max_steer_ = 1.0;
};

#endif // CARLA1S_ROS_CARLA1S_NAVIGATION_CARLA1S_PLANNING_CARLA1S_PATH_TRACKING_SRC_LAT_CONTROLLER_STANLEY_H_
