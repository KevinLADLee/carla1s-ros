#ifndef CARLA1S_ROS_CARLA_NAVIGATION_CARLA_PATH_TRACKING_SRC_LON_CONTROLLER__PID_H_
#define CARLA1S_ROS_CARLA_NAVIGATION_CARLA_PATH_TRACKING_SRC_LON_CONTROLLER__PID_H_

#include "common/vehicle_controller_base.h"
#include "common/pid_impl.h"

using namespace carla1s;

class PidLonController : public VehicleController{
 public:
  PidLonController();

  void Reset(const DirectedPath2dPtr &directed_path_ptr) override;

  NodeState RunStep(const VehicleState &vehicle_state,
                    const double &target_speed,
                    const double &dt,
                    double &throttle) override;

  double GetSpeedError();

  double GetStationError();

 private:
  double GetLookaheadDist() const;

  NodeState ComputeLonErrors(double &error);

 private:
  std::unique_ptr<PIDImpl<double>> speed_controller_;
  std::unique_ptr<PIDImpl<double>> station_controller_;

  double max_throttle = 1.0;
  double target_speed_ = 0.0;

  double max_fwd_lookahead_dist = 2.5;
  double max_bck_lookahead_dist = 1.0;

  // {Kp, Ki, Kd, max_speed, min_speed}
  const std::vector<double> station_pid_param_fwd_ = {5.5, 0.0, 0.0, 15.0, 0.0};
  const std::vector<double> station_pid_param_bck_ = {2.5, 0.0, 0.0, 3.0, 0.0};

  // {Kp, Ki, Kd, max_acc, min_acc}
  const std::vector<double> speed_pid_param_fwd_ =  {0.34, 0.000004, 0.0000004, 1.0, 0.0};
  const std::vector<double> speed_pid_param_bck_ = {0.2, 0, 0, 1.0, 0.0};

};



#endif