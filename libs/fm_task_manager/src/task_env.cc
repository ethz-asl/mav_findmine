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

#include "fm_task_manager/task_env.h"

#include <dji_sdk/dji_sdk.h>
#include <mav_planning_msgs/PolynomialTrajectory4D.h>
#include <std_srvs/Empty.h>

namespace fm_task_manager {

TaskEnv::TaskEnv(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
    : task_manager_lib::TaskEnvironment(nh_private),
      nh_(nh),
      nh_private_(nh_private) {
  getRosParameters();
  subscribeTopics();
  initializeStates();
  advertiseTopics();
}

void TaskEnv::getRosParameters() {
  nh_private_.param("message_timeout_s", message_timeout_s_, 1.0);
}

const size_t kDefaultQueueSize = 1;
void TaskEnv::subscribeTopics() {
  display_mode_sub_ = nh_.subscribe("dji_sdk/display_mode", kDefaultQueueSize,
                                    &TaskEnv::receiveDisplayMode, this);
  flight_status_sub_ = nh_.subscribe("dji_sdk/flight_status", kDefaultQueueSize,
                                     &TaskEnv::receiveFlightStatus, this);
}

void TaskEnv::initializeStates() {
  display_mode_.data = DJISDK::DisplayMode::MODE_P_GPS;
  flight_status_.data = DJISDK::FlightStatus::STATUS_ON_GROUND;
}

void TaskEnv::advertiseTopics() {
  const int kQueueSize = 1;
  const bool kLatch = false;
  trajectory_pub_ = nh_.advertise<mav_planning_msgs::PolynomialTrajectory4D>(
      "command/polynomial_trajectory", kQueueSize, kLatch);
}

void TaskEnv::receiveDisplayMode(const std_msgs::UInt8ConstPtr& msg) {
  display_mode_ = *msg;
}

void TaskEnv::receiveFlightStatus(const std_msgs::UInt8ConstPtr& msg) {
  flight_status_ = *msg;
}

bool TaskEnv::obtainCtrlAuthority() {
  ros::ServiceClient obtain_ctrl_authority_client =
      nh_.serviceClient<std_srvs::Empty>("obtain_control_authority");
  std_srvs::Empty srv;
  return obtain_ctrl_authority_client.call(srv);
}

bool TaskEnv::releaseCtrlAuthority() {
  ros::ServiceClient release_ctrl_authority_client =
      nh_.serviceClient<std_srvs::Empty>("release_control_authority");
  std_srvs::Empty srv;
  return release_ctrl_authority_client.call(srv);
}

}  // namespace fm_task_manager
