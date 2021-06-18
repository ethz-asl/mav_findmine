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

#include "fm_task_manager/tasks/execute_single_trajectory.h"

#include <limits>

#include <eigen_conversions/eigen_msg.h>
#include <mav_trajectory_generation/trajectory.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>
#include <task_manager_msgs/TaskStatus.h>

namespace tmm = task_manager_msgs;

namespace fm_task_manager {
namespace tasks {

ExecuteSingleTrajectory::ExecuteSingleTrajectory(tml::TaskDefinitionPtr def,
                                                 tml::TaskEnvironmentPtr env)
    : Parent(def, env) {}

void ExecuteSingleTrajectory::subscribeTopics() {
  const size_t kQueueSize = 1;
  local_position_sub_ =
      env->nh_.subscribe("dji_sdk/velocity", kQueueSize,
                         &ExecuteSingleTrajectory::receiveVelocity, this);

  position_error_sub_ =
      env->nh_.subscribe("position_controller/pos_error", kQueueSize,
                         &ExecuteSingleTrajectory::receivePositionError, this);

  ctrl_auth_sub_ =
      env->nh_.subscribe("position_controller/ctrl_authority", kQueueSize,
                         &ExecuteSingleTrajectory::receiveCtrlAuthority, this);
}

void ExecuteSingleTrajectory::receiveVelocity(
    const geometry_msgs::Vector3StampedConstPtr& msg) {
  Eigen::Vector3d velocity;
  tf::vectorMsgToEigen(msg->vector, velocity);
  abs_velocity_ = velocity.norm();
}

void ExecuteSingleTrajectory::receivePositionError(
    const std_msgs::Float32ConstPtr& msg) {
  tracking_error_m_ = msg->data;
}

void ExecuteSingleTrajectory::receiveCtrlAuthority(
    const std_msgs::BoolConstPtr& msg) {
  has_ctrl_ = msg->data;
}

tml::TaskIndicator ExecuteSingleTrajectory::initialise() {
  // States:
  tracking_error_m_ = std::numeric_limits<double>::max();
  abs_velocity_ = std::numeric_limits<double>::max();
  has_ctrl_ = false;

  // 1. Obtain trajectory.
  if (env->trajectory_.empty()) {
    ROS_ERROR("Empty trajectory.");
    return tmm::TaskStatus::TASK_INITIALISATION_FAILED;
  }

  // 2. Update goal information.
  trajectory_duration_ = ros::Duration(env->trajectory_.getMaxTime());
  subscribeTopics();

  // 3. Obtain control authority.
  ROS_INFO("Obtaining control authority.");
  if (!env->obtainCtrlAuthority()) {
    ROS_WARN("Could not obtain control authority.");
    return tmm::TaskStatus::TASK_INITIALISATION_FAILED;
  } else {
    has_ctrl_ = true;
  }

  // 4. Send trajectory.
  mav_planning_msgs::PolynomialTrajectory4D trajectory_ENU;
  if (!mav_trajectory_generation::trajectoryToPolynomialTrajectoryMsg(
          env->trajectory_, &trajectory_ENU)) {
    ROS_ERROR("Failed to convert full trajectory to message.");
    return tmm::TaskStatus::TASK_INITIALISATION_FAILED;
  }
  trajectory_ENU.header.stamp = ros::Time::now();
  trajectory_ENU.header.frame_id = "ground_ENU";
  env->trajectory_pub_.publish(trajectory_ENU);
  trajectory_start_time_ = ros::Time::now();

  return tmm::TaskStatus::TASK_INITIALISED;
}

tml::TaskIndicator ExecuteSingleTrajectory::iterate() {
  // 5. Check if finished.
  if (!has_ctrl_) {
    ROS_ERROR("No control authority.");
    return tmm::TaskStatus::TASK_FAILED;
  } else if ((ros::Time::now() - trajectory_start_time_) >
                 trajectory_duration_ &&
             tracking_error_m_ < cfg.ball_radius &&
             abs_velocity_ < cfg.terminal_vel) {
    ROS_INFO("Trajectory completed.");
    return tmm::TaskStatus::TASK_COMPLETED;
  }
  return tmm::TaskStatus::TASK_RUNNING;
}

tml::TaskIndicator ExecuteSingleTrajectory::terminate() {
  // 6. Release contol authority.
  ROS_INFO("Releasing control authority.");
  if (!env->releaseCtrlAuthority()) {
    ROS_ERROR("Could not release control authority.");
    return tmm::TaskStatus::TASK_FAILED;
  }

  return tmm::TaskStatus::TASK_TERMINATED;
}

const bool kIsPeriodic = true;
ExecuteSingleTrajectoryFactory::ExecuteSingleTrajectoryFactory(
    tml::TaskEnvironmentPtr env)
    : Parent("ExecuteSingleTrajectory", "Execute a single trajectory.",
             kIsPeriodic, env) {}

}  // namespace tasks
}  // namespace fm_task_manager

using namespace task_manager_lib;
DYNAMIC_TASK(fm_task_manager::tasks::ExecuteSingleTrajectoryFactory);
