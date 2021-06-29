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


#include "pid.h"

using namespace std;


PID::PID(double Kp, double Kd, double Ki, double max, double min, double dt)
{
  pimpl = std::make_unique<PIDImpl<double>>(Kp,Kd,Ki,max,min,dt);
}

PID::~PID() {

}


double PID::RunStep(const double &target_speed,
                    const double &vehicle_speed) {
  std::lock_guard<std::mutex> lock_guard(pid_mutex_);
  return pimpl->RunStep(target_speed, vehicle_speed);
}

void PID::ResetParam(double Kp, double Kd, double Ki, double max, double min, double dt) {
  std::lock_guard<std::mutex> lock_guard(pid_mutex_);
  pimpl.reset();
  pimpl = std::make_unique<PIDImpl<double>>(Kp, Ki, Kd, max, min, dt);
}






#endif