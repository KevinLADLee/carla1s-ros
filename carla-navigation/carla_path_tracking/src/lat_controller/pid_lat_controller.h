
#ifndef CARLA1S_ROS_CARLA_NAVIGATION_CARLA_PATH_TRACKING_SRC_IMPL_PID_LAT_CONTROLLER_H_
#define CARLA1S_ROS_CARLA_NAVIGATION_CARLA_PATH_TRACKING_SRC_IMPL_PID_LAT_CONTROLLER_H_

#include "lat_controller_base.h"
#include "common/pid_impl.h"

class PidLatController : public LatController{
 public:
  PidLatController();

  ~PidLatController() = default;

  double ComputeLatErrors(const Pose2dPtr &vehicle_pose_ptr,
                          const Path2dPtr &waypoints_ptr) override;

  double RunStep(const Pose2dPtr &vehicle_pose_ptr,
                 const Path2dPtr &waypoints_ptr,
                 const double &vehicle_speed,
                 const double &dt) override;

  int SetDrivingDirection(const DrivingDirection &driving_direction) override;

 private:
  std::unique_ptr<PIDImpl<double>> pid_ptr_;
  const std::vector<double> pid_param_fwd_ = {4.5, 0, 0.5, 1.0, -1.0};
  const std::vector<double> pid_param_bck_ = {35.0, 0, 0.0, 1.0, -1.0};
  double center_ref_dist_ = 1.5; // Distance from vehicle centre to front axle (m)
  double center_bck_dist_ = 0.5;
};

#endif //CARLA1S_ROS_CARLA_NAVIGATION_CARLA_PATH_TRACKING_SRC_IMPL_PID_LAT_CONTROLLER_H_
