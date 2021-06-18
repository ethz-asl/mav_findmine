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

#ifndef FM_CONTROL_PID_CONTROLLER_H
#define FM_CONTROL_PID_CONTROLLER_H

#include <ros/ros.h>

namespace fm_control {

class PidController {
 public:
  PidController();
  PidController(const PidController&) = default;
  ~PidController() = default;

  void reset();
  // Set the internal state of the controller for bump-less transfer.
  void setControllerState(double u, double r, double y, ros::Time t);

  // Compute the PID output signal
  double control(double r, double y, ros::Time t);
  double control(double r, double dr, double y, double dy, ros::Time t);
  // Feedback the saturated control value to prevent integrator windup.
  void setSaturatedOutput(double u_saturated);

  // Change control parameters.
  void setParameters(double Kp, double Ti, double Td);
  void setSetpointWeights(double b, double c);

  double getControlOutput() const;

 private:
  // Reset the integrator such that the gains and the control output are
  // consistent. Allows smooth transfers for control gain changes.
  void integratorBackCalculation();

  // Stored time.
  ros::Time t_;

  // Keeps track if the controller was resetted.
  bool reset_flag_;

  // Stored signals.
  double r_previous_;
  double y_previous_;
  double dr_previous_;
  double dy_previous_;
  double i_previous_;
  double u_previous_;
  double u_saturated_previous_;
  // Gains, setpoint weights.
  double Kp_;
  double Ki_;
  double Kd_;
  double Tt_;
  double b_;
  double c_;
};

}  // namespace fm_control

#endif  // FM_CONTROL_PID_CONTROLLER_H
