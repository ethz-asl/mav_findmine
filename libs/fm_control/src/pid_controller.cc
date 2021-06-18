/*
MIT License

Copyright (c) 2020 Rik Baehnemann, ASL, ETH Zurich, Switzerland

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include "fm_control/pid_controller.h"

namespace fm_control {

PidController::PidController()
    : Kp_(0.0), Ki_(0.0), Kd_(0.0), b_(1.0), c_(0.0) {
  reset();
  // By default, intialize a P-controller with gain 1.
  setParameters(1.0, 0.0, 0.0);
  setSetpointWeights(1.0, 0.0);
};

void PidController::reset() {
  reset_flag_ = true;
  r_previous_ = 0.0;
  y_previous_ = 0.0;
  dr_previous_ = 0.0;
  dy_previous_ = 0.0;
  i_previous_ = 0.0;
  u_previous_ = 0.0;
  u_saturated_previous_ = 0.0;
}

void PidController::setControllerState(double u, double r, double y,
                                       ros::Time t) {
  // Update the proportional, integral and derivative signal.
  control(r, y, t);
  // Overwrite the control output.
  u_previous_ = u;
  u_saturated_previous_ = u;
  // Adjust the integral signal to be consistent with the overwritten control
  // output.
  integratorBackCalculation();
}

double PidController::control(double r, double y, ros::Time t) {
  double dt = (t - t_).toSec();
  double dr;
  double dy;
  if (reset_flag_) {
    // If the controller was reset, do not use the previous values to compute
    // the derivative signals but set them to zero.
    dr = dy = 0.0;
  } else {
    dr = (r - r_previous_) / dt;
    dy = (y - y_previous_) / dt;
  }
  return control(r, dr, y, dy, t);
}

double PidController::control(double r, double dr, double y, double dy,
                              ros::Time t) {
  double dt = (t - t_).toSec();

  double proportional = b_ * r - y;
  double integral = i_previous_ + (r_previous_ - y_previous_) * dt;
  double derivative = c_ * dr - dy;

  // Apply anti reset windup.
  double Kt = (Tt_ == 0.0) ? 0.0 : 1.0 / Tt_;
  integral += Kt * dt * (u_saturated_previous_ - u_previous_);

  double u = Kp_ * proportional + Ki_ * integral + Kd_ * derivative;
  r_previous_ = r;
  y_previous_ = y;
  dr_previous_ = dr;
  dy_previous_ = dy;
  i_previous_ = integral;
  t_ = t;
  u_previous_ = u;
  // Assume no saturation until SetSaturatedOutput() is called.
  u_saturated_previous_ = u;

  return u;
}

void PidController::setSaturatedOutput(double u_saturated) {
  u_saturated_previous_ = u_saturated;
}

void PidController::setParameters(double Kp, double Ti, double Td) {
  Kp_ = Kp;
  Ki_ = (Ti == 0.0) ? 0.0 : Kp / Ti;
  Kd_ = Kp * Td;
  if (Ti == 0.0) {
    Tt_ = 0.0;
  } else if (Td == 0.0) {
    Tt_ = 2.0 * Ti;
  } else {
    Tt_ = std::sqrt(Td * Ti);
  }
  integratorBackCalculation();
}

double PidController::getControlOutput() const { return u_previous_; }

void PidController::setSetpointWeights(double b, double c) {
  b_ = b;
  c_ = c;
  integratorBackCalculation();
}

void PidController::integratorBackCalculation() {
  if (Ki_ == 0.0) {
    i_previous_ = 0.0;
  } else {
    double proportional = b_ * r_previous_ - y_previous_;
    double derivative = c_ * dr_previous_ - dy_previous_;
    i_previous_ = (u_previous_ - Kp_ * proportional - Kd_ * derivative) / Ki_;
  }
}

}  // namespace fm_control
