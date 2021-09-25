#ifndef CARLA1S_ROS_CARLA_NAVIGATION_CARLA_PATH_TRACKING_SRC_LON_CONTROLLER__PID_H_
#define CARLA1S_ROS_CARLA_NAVIGATION_CARLA_PATH_TRACKING_SRC_LON_CONTROLLER__PID_H_

#include "lon_controller_base.h"
#include "common/pid_impl.h"

using namespace carla1s;

class PidLonController : public LonController{
 public:
  PidLonController();

  double RunStep(const Pose2dPtr &vehicle_pose_ptr,
                 const Path2dPtr &waypoints_ptr,
                 const double &vehicle_speed,
                 const double &dt) override;

  double RunStep(const double &target_speed,
                 const double &vehicle_speed,
                 const double &dt) override;

  int SetDrivingDirection(const DrivingDirection &driving_direction) override;

  double GetSpeedError();

  double GetStationError();


 private:
  std::unique_ptr<PIDImpl<double>> speed_controller_;
  std::unique_ptr<PIDImpl<double>> station_controller_;

  // {Kp, Ki, Kd, max_speed, min_speed}
  const std::vector<double> station_pid_param_fwd_ = {8.5, 0.0, 0.001, 15.0, 0.0};
  const std::vector<double> station_pid_param_bck_ = {2.5, 0.0, 0.0, 3.0, 0.0};

  // {Kp, Ki, Kd, max_acc, min_acc}
  const std::vector<double> speed_pid_param_fwd_ =  {0.34, 0.000004, 0.0000004, 1.0, 0.0};
  const std::vector<double> speed_pid_param_bck_ = {0.2, 0, 0, 1.0, 0.0};

};



#endif