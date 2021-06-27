//
// Created by kevinlad on 2021/6/28.
//

#include "pid.h"

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

#include <iostream>
#include <cmath>
#include "pid.h"

using namespace std;

class PIDImpl
{
 public:
  PIDImpl( double dt, double max, double min, double Kp, double Kd, double Ki );
  ~PIDImpl() = default;
  double RunStep( double setpoint, double pv );

 private:
  double dt_;
  double max_;
  double min_;
  double Kp_;
  double Kd_;
  double Ki_;
  double pre_error_;
  double integral_;
};


PID::PID( double dt, double max, double min, double Kp, double Kd, double Ki )
{
  pimpl = new PIDImpl(dt,max,min,Kp,Kd,Ki);
}
double PID::RunStep( double setpoint, double pv )
{
  return pimpl->RunStep(setpoint,pv);
}
PID::~PID()
{
  delete pimpl;
}


/**
 * Implementation
 */
PIDImpl::PIDImpl(double dt, double max, double min, double kp, double kd, double ki)
    : dt_(dt), max_(max), min_(min), Kp_(kp), Kd_(kd), Ki_(ki), pre_error_(0), integral_(0) {}


double PIDImpl::RunStep( double setpoint, double pv )
{

  // Calculate error
  double error = setpoint - pv;

  // Proportional term
  double Pout = Kp_ * error;

  // Integral term
  integral_ += error * dt_;
  double Iout = Ki_ * integral_;

  // Derivative term
  double derivative = (error - pre_error_) / dt_;
  double Dout = Kd_ * derivative;

  // Calculate total output
  double output = Pout + Iout + Dout;

  // Restrict to max/min
  if( output > max_ )
    output = max_;
  else if( output < min_ )
    output = min_;

  // Save error to previous error
  pre_error_ = error;

  return output;
}



#endif