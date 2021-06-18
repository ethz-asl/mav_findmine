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

#include "fm_task_manager/tasks/take_off.h"

#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/dji_sdk.h>
#include <fm_comm/dji_interface.h>
#include <task_manager_msgs/TaskStatus.h>

namespace tmm = task_manager_msgs;

namespace fm_task_manager {
namespace tasks {

TakeOff::TakeOff(tml::TaskDefinitionPtr def, tml::TaskEnvironmentPtr env)
    : Parent(def, env) {}

tml::TaskIndicator TakeOff::initialise() {
  // Check already in air.
  if (env->flight_status_.data == DJISDK::FlightStatus::STATUS_IN_AIR) {
    ROS_WARN("UAV already in air.");
    return tmm::TaskStatus::TASK_INITIALISATION_FAILED;
  }

  // Actual take off.
  ROS_INFO("Calling DJI take off.");
  if (!callDjiTakeOff()) {
    ROS_WARN("Cannot engage auto take off.");
    return tmm::TaskStatus::TASK_INITIALISATION_FAILED;
  }
  ROS_INFO("Starting motors and ascending.");

  return tmm::TaskStatus::TASK_INITIALISED;
}

tml::TaskIndicator TakeOff::iterate() {
  if (env->display_mode_.data != DJISDK::DisplayMode::MODE_AUTO_TAKEOFF &&
      env->flight_status_.data == DJISDK::FlightStatus::STATUS_IN_AIR) {
    ROS_INFO("Take off completed.");
    return tmm::TaskStatus::TASK_COMPLETED;
  }

  return tmm::TaskStatus::TASK_RUNNING;
}

bool TakeOff::callDjiTakeOff() {
  ros::ServiceClient drone_task_ctrl_client =
      env->nh_.serviceClient<dji_sdk::DroneTaskControl>(
          "dji_sdk/drone_task_control");

  dji_sdk::DroneTaskControl::Request dtc_req;
  dtc_req.task = dji_sdk::DroneTaskControl::Request::TASK_TAKEOFF;
  return fm_comm::callDjiTask<dji_sdk::DroneTaskControl>(
      dtc_req, drone_task_ctrl_client);
}

const bool kIsPeriodic = true;
TakeOffFactory::TakeOffFactory(tml::TaskEnvironmentPtr env)
    : Parent("TakeOff", "Start motors, get control, and take off.", kIsPeriodic,
             env) {}

}  // namespace tasks
}  // namespace fm_task_manager

using namespace task_manager_lib;
DYNAMIC_TASK(fm_task_manager::tasks::TakeOffFactory);
