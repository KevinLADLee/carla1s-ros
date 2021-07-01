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

#ifndef _PID_H_
#define _PID_H_

#include <iostream>
#include <cmath>
#include <cassert>
#include <mutex>
#include "planner_common.h"
#include "path_tracking_base.h"

template <typename T>
class PIDImpl
{
 public:
  PIDImpl(T kp, T kd, T ki, T max, T min, T dt)
      : dt_(dt), max_(max), min_(min), Kp_(kp), Kd_(kd), Ki_(ki), pre_error_(0), integral_(0) {}


  T RunStep( T target, T current)
  {

    // Calculate error
    T error = target - current;

    // Proportional term
    T Pout = Kp_ * error;

    // Integral term
    integral_ += error * dt_;
    integral_ = clip(integral_, min_integral_, max_integral_);
    T Iout = Ki_ * integral_;

    // Derivative term
    assert(dt_ > 0);
    T derivative = (error - pre_error_) / dt_;
    T Dout = Kd_ * derivative;

    // Calculate total output
    T output = Pout + Iout + Dout;

    // Restrict to max/min
    output = clip(output, min_, max_);

    // Save error to previous error
    pre_error_ = error;

    return output;
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
  T max_integral_;
  T min_integral_;
};

class PID : public LongitudinalController{
 public:
  // Kp -  proportional gain
  // Ki -  Integral gain
  // Kd -  derivative gain
  // dt -  loop interval time
  // max - maximum value of manipulated variable
  // min - minimum value of manipulated variable
  PID(double Kp = 1.0, double Kd = 0.0, double Ki = 0.0, double max = 1.0, double min = 0.0, double dt = 1.0);

  virtual ~PID();

  double RunStep(const double &target_speed,
                 const double &vehicle_speed) override;

  void ResetParam(double Kp, double Kd, double Ki, double max = 1.0, double min = 0.0, double dt = 1.0);

 private:
  std::unique_ptr<PIDImpl<double>> pimpl;
  std::mutex pid_mutex_;
};



#endif