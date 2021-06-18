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

#include <cstdlib>

#include "fm_task_manager/tasks/logging.h"

#include <dji_sdk/dji_sdk.h>

#include <task_manager_msgs/TaskStatus.h>

namespace tmm = task_manager_msgs;

namespace fm_task_manager {
namespace tasks {

Logging::Logging(tml::TaskDefinitionPtr def, tml::TaskEnvironmentPtr env)
    : Parent(def, env) {}
tml::TaskIndicator Logging::initialise() {
  // Reset internal state.
  taken_off_ = false;

  // Start logging.
  std::string dir = cfg.directory;
  ROS_INFO("Start logging %s", dir.c_str());

  // TODO(rikba): Do not use system() but use rosbag recorder API.
  if (!system(NULL)) {
    ROS_ERROR("Command process is not available.");
    return tmm::TaskStatus::TASK_INITIALISATION_FAILED;
  }

  std::string cmd = "roslaunch fm_missions record_sensors.launch bag_base:=";
  cmd += cfg.directory;
  cmd += "/";
  cmd += " record_cam:=";
  if (cfg.record_cam) {
    cmd += "True";
  } else {
    cmd += "False";
  }
  cmd += " &";
  system(cmd.c_str());

  return tmm::TaskStatus::TASK_INITIALISED;
}

tml::TaskIndicator Logging::iterate() {
  // Check if in air.
  if (taken_off_ &&
      env->flight_status_.data == DJISDK::FlightStatus::STATUS_STOPPED) {
    ROS_INFO("UAV back on ground.");
    return tmm::TaskStatus::TASK_COMPLETED;
  } else if (env->flight_status_.data == DJISDK::FlightStatus::STATUS_IN_AIR) {
    taken_off_ = true;
  }

  return tmm::TaskStatus::TASK_RUNNING;
}

tml::TaskIndicator Logging::terminate() {
  // Stop logging.
  ROS_INFO("Logging complete.");
  if (!system(NULL)) {
    ROS_ERROR("Command process is not available.");
    return tmm::TaskStatus::TASK_FAILED;
  }

  // TODO(rikba): Do not use system() but use rosbag recorder API.
  // TODO(rikba): For some reason the logging node gets started in a double
  // namespace.
  std::string cmd = "rosnode kill /";
  cmd += getenv("USER");
  cmd += "/";
  cmd += getenv("USER");
  cmd += "/rosbag_record_sensors";
  system(cmd.c_str());

  return tmm::TaskStatus::TASK_TERMINATED;
}

const bool kIsPeriodic = true;
LoggingFactory::LoggingFactory(tml::TaskEnvironmentPtr env)
    : Parent("Logging", "Rosbag logging while in air.", kIsPeriodic, env) {}

}  // namespace tasks
}  // namespace fm_task_manager

using namespace task_manager_lib;
DYNAMIC_TASK(fm_task_manager::tasks::LoggingFactory);
