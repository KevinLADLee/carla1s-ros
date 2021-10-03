/**
 * Copyright 2019 Bradley J. Snyder <snyder.bradleyj@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef _PID_SOURCE_
#define _PID_SOURCE_


#include "pid_lon_controller.h"

using namespace std;

PidLonController::PidLonController()
{
  station_controller_ = std::make_unique<PIDImpl<double>>(station_pid_param_fwd_);
  speed_controller_ = std::make_unique<PIDImpl<double>>(speed_pid_param_fwd_);
}

double PidLonController::RunStep(const double &target_speed,
                    const double &vehicle_speed,
                    const double &dt) {
//  std::cout << "target_speed: " << target_speed << " vehicle_speed: " << vehicle_speed << std::endl;
  return speed_controller_->RunStep(target_speed, vehicle_speed, dt);
}

double PidLonController::RunStep(const Pose2dPtr &vehicle_pose_ptr,
                                 const Path2dPtr &waypoints_ptr,
                                 const double &vehicle_speed,
                                 const double &dt) {

  SetWaypoints(waypoints_ptr);

  double lon_error = std::abs(ComputeLonErrors(vehicle_pose_ptr, waypoints_ptr));
  SetLatestError(lon_error);

  auto speed_offset = station_controller_->RunStep(lon_error, dt);

  auto throttle = speed_controller_->RunStep(speed_offset, vehicle_speed, dt);
  return throttle;
}

double PidLonController::GetSpeedError() {
  return speed_controller_->GetPreError();
}
double PidLonController::GetStationError() {
  return station_controller_->GetPreError();
}


int PidLonController::SetDrivingDirection(const DrivingDirection &driving_direction) {
  if(driving_direction != GetDrivingDirection()){
    if(driving_direction == DrivingDirection::BACKWARDS){
      station_controller_->ResetParam(station_pid_param_bck_);
      speed_controller_->ResetParam(speed_pid_param_bck_);
    }else{
      station_controller_->ResetParam(station_pid_param_fwd_);
      speed_controller_->ResetParam(speed_pid_param_fwd_);
    }
  }
  return VehicleController::SetDrivingDirection(driving_direction);
}

void PidLonController::SetTargetSpeed(double target_speed) {
  LonController::SetTargetSpeed(target_speed);
  station_controller_->SetMax(target_speed);
}
#endif