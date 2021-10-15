
#ifndef CARLA1S_ROS_CARLA_NAVIGATION_CARLA_PATH_TRACKING_SRC_IMPL_PID_LAT_CONTROLLER_H_
#define CARLA1S_ROS_CARLA_NAVIGATION_CARLA_PATH_TRACKING_SRC_IMPL_PID_LAT_CONTROLLER_H_

#include "common/vehicle_controller_base.h"
#include "common/pid_impl.h"

class PidLatController : public VehicleController{
 public:
  PidLatController();

  ~PidLatController() = default;

  void Reset(const DirectedPath2dPtr &directed_path_ptr) override;

  NodeState RunStep(const VehicleState &vehicle_state,
                    const double &target_speed,
                    const double &dt,
                    double &steer) override;

  NodeState ComputeLatErrors(double &error);

 private:
  std::unique_ptr<PIDImpl<double>> pid_ptr_;
  const std::vector<double> pid_param_fwd_ = {4.5, 0, 0.5, 1.0, -1.0};
  const std::vector<double> pid_param_bck_ = {35.0, 0, 0.0, 1.0, -1.0};
  double center_ref_dist_ = 1.5; // Distance from vehicle centre to front axle (m)
  double center_bck_dist_ = 0.5;
};

#endif //CARLA1S_ROS_CARLA_NAVIGATION_CARLA_PATH_TRACKING_SRC_IMPL_PID_LAT_CONTROLLER_H_
