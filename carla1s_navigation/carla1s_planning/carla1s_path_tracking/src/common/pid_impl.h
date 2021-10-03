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


#ifndef CARLA1S_ROS_CARLA_NAVIGATION_CARLA_PATH_TRACKING_SRC_IMPL_PID_IMPL_H_
#define CARLA1S_ROS_CARLA_NAVIGATION_CARLA_PATH_TRACKING_SRC_IMPL_PID_IMPL_H_

#include "carla_nav_math/math_utils.h"
using namespace carla1s;

template <typename T>
class PIDImpl
{
 public:
  explicit PIDImpl(const std::vector<T> &params){
    ResetParam(params);
  }

  PIDImpl(T kp, T ki, T kd, T max, T min)
      : max_(max), min_(min), Kp_(kp), Ki_(ki), Kd_(kd), pre_error_(0), integral_(0) {}

  void ResetParam(const std::vector<T> &params){
    assert(params.size() == 5);
    Kp_ = params[0];
    Ki_ = params[1];
    Kd_ = params[2];
    max_ = params[3];
    min_ = params[4];
    pre_error_ = 0;
    integral_ = 0;
  }

  void SetMax(T max){
    max_ = max;
  };

  T RunStep( T target, T current, T dt)
  {

    // Calculate error
    T error = target - current;

    return RunStep(error, dt);
  }

  T RunStep(T error, T dt)
  {
    // Proportional term
    T Pout = Kp_ * error;

    // Integral term
    integral_ += error * dt;
    integral_ = math::Clip(integral_, min_integral_, max_integral_);
    T Iout = Ki_ * integral_;

    // Derivative term
    assert(dt > 0);
    T derivative = (error - pre_error_) / dt;
    T Dout = Kd_ * derivative;

    // Calculate total output
    T output = Pout + Iout + Dout;

    // Restrict to max/min
    output = math::Clip(output, min_, max_);

    // Save error to previous error
    pre_error_ = error;

    return output;
  }

  T GetPreError(){
    return pre_error_;
  }

 public:
  T dt_;
  T max_;
  T min_;
  T Kp_;
  T Kd_;
  T Ki_;
  T pre_error_;
  T integral_;
  T max_integral_ = 1000.0;
  T min_integral_ = -1000.0;
};


#endif //CARLA1S_ROS_CARLA_NAVIGATION_CARLA_PATH_TRACKING_SRC_IMPL_PID_IMPL_H_
