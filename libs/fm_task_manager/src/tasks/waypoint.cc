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

#include "fm_task_manager/tasks/waypoint.h"

#include <limits>

#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <mav_msgs/common.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <ros/topic.h>
#include <std_msgs/Float32.h>
#include <task_manager_msgs/TaskStatus.h>
#include <Eigen/Core>

namespace tmm = task_manager_msgs;

namespace fm_task_manager {
namespace tasks {

Waypoint::Waypoint(tml::TaskDefinitionPtr def, tml::TaskEnvironmentPtr env)
    : Parent(def, env) {}

const bool kQueueSize = 1;
void Waypoint::subscribeTopics() {
  velocity_sub_ = env->nh_.subscribe("dji_sdk/velocity", kQueueSize,
                                     &Waypoint::receiveVelocity, this);
  position_error_sub_ =
      env->nh_.subscribe("position_controller/pos_error", kQueueSize,
                         &Waypoint::receivePositionError, this);

  ctrl_auth_sub_ =
      env->nh_.subscribe("position_controller/ctrl_authority", kQueueSize,
                         &Waypoint::receiveCtrlAuthority, this);
}

void Waypoint::advertiseTopics() {
  const bool kLatchTopic = false;
  waypoint_pub_ = env->nh_.advertise<geometry_msgs::PoseStamped>(
      "command/pose", kQueueSize, kLatchTopic);
}

void Waypoint::receiveVelocity(
    const geometry_msgs::Vector3StampedConstPtr& msg) {
  Eigen::Vector3d velocity;
  tf::vectorMsgToEigen(msg->vector, velocity);
  abs_velocity_ = velocity.norm();
}

void Waypoint::receiveCtrlAuthority(const std_msgs::BoolConstPtr& msg) {
  has_ctrl_ = msg->data;
}

void Waypoint::receivePositionError(const std_msgs::Float32ConstPtr& msg) {
  tracking_error_m_ = msg->data;
}

tml::TaskIndicator Waypoint::initialise() {
  subscribeTopics();
  advertiseTopics();

  // Initialize states.
  tracking_error_m_ = std::numeric_limits<double>::max();
  abs_velocity_ = std::numeric_limits<double>::max();
  has_ctrl_ = false;

  // Goal type.
  // TODO(rikba, stlucas): Access enum entry from config
  switch (cfg.goal_type) {
    case SET_ALL:  // probably cfg,GOAL_TYPE_SET_ALL
      ROS_INFO("GOAL_TYPE_SET_ALL");
      break;
    case FIXED_YAW:
      ROS_INFO("GOAL_TYPE_FIXED_YAW");
      if (!getCurrentYaw(&cfg.goal_yaw))
        return tmm::TaskStatus::TASK_INITIALISATION_FAILED;
      break;
    case CHANGE_ALTITUDE:
      ROS_INFO("GOAL_TYPE_CHANGE_ALTITUDE");
      if (!getCurrentXY(&cfg.goal_x, &cfg.goal_y) ||
          !getCurrentYaw(&cfg.goal_yaw))
        return tmm::TaskStatus::TASK_INITIALISATION_FAILED;
      break;
    case TRAJECTORY_START_XY:
      ROS_INFO("GOAL_TYPE_TRAJECTORY_START_XY");
      if (!getTrajectoryStartXY(&cfg.goal_x, &cfg.goal_y) ||
          !getCurrentYaw(&cfg.goal_yaw))
        return tmm::TaskStatus::TASK_INITIALISATION_FAILED;
      break;
    case TRAJECTORY_START_XYZ:
      ROS_INFO("GOAL_TYPE_TRAJECTORY_START");
      if (!getTrajectoryStartXYZ(&cfg.goal_x, &cfg.goal_y, &cfg.goal_z) ||
          !getCurrentYaw(&cfg.goal_yaw))
        return tmm::TaskStatus::TASK_INITIALISATION_FAILED;
      break;

    case TAKE_OFF_POINT:
      ROS_INFO("GOAL_TYPE_TAKEOFF_POINT");
      if (!getTakeoffXY(&cfg.goal_x, &cfg.goal_y) ||
          !getCurrentYaw(&cfg.goal_yaw))
        return tmm::TaskStatus::TASK_INITIALISATION_FAILED;
      break;

    default:
      ROS_WARN("Invalid waypoint goal type.");
      return tmm::TaskStatus::TASK_INITIALISATION_FAILED;
  }

  // Obtain control authority.
  ROS_INFO("Obtaining control authority.");
  if (!env->obtainCtrlAuthority()) {
    ROS_WARN("Could not obtain control authority.");
    return tmm::TaskStatus::TASK_INITIALISATION_FAILED;
  } else {
    has_ctrl_ = true;
  }

  return tmm::TaskStatus::TASK_INITIALISED;
}

tml::TaskIndicator Waypoint::terminate() {
  // Release contol authority.
  ROS_INFO("Releasing control authority.");
  if (!env->releaseCtrlAuthority()) {
    ROS_ERROR("Could not release control authority.");
    return tmm::TaskStatus::TASK_FAILED;
  }

  return tmm::TaskStatus::TASK_TERMINATED;
}

void Waypoint::publishWaypoint() {
  ROS_INFO_STREAM_ONCE("Sending waypoint in ENU frame with x [m]: "
                       << cfg.goal_x << " y [m]: " << cfg.goal_y << " z [m]: "
                       << cfg.goal_z << " yaw [rad]: " << cfg.goal_yaw);

  mav_msgs::EigenTrajectoryPoint wp;
  wp.position_W = Eigen::Vector3d(cfg.goal_x, cfg.goal_y, cfg.goal_z);
  wp.setFromYaw(cfg.goal_yaw);

  geometry_msgs::PoseStamped msg;
  mav_msgs::msgPoseStampedFromEigenTrajectoryPoint(wp, &msg);
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "/local";

  waypoint_pub_.publish(msg);
}

tml::TaskIndicator Waypoint::iterate() {
  // Send waypoint to controller.
  publishWaypoint();

  // Check if terminated when inside ball.
  if (!has_ctrl_) {
    ROS_ERROR("No control authority.");
    return tmm::TaskStatus::TASK_FAILED;
  } else if (tracking_error_m_ < cfg.ball_radius &&
             abs_velocity_ < cfg.terminal_vel) {
    ROS_INFO_STREAM(
        "Reached waypoint with tracking error: " << tracking_error_m_);
    return tmm::TaskStatus::TASK_COMPLETED;
  }

  return tmm::TaskStatus::TASK_RUNNING;
}

bool Waypoint::getCurrentXY(double* x, double* y) {
  ROS_ASSERT(x == nullptr);
  ROS_ASSERT(y);

  // Receive current x,y.
  geometry_msgs::PointStampedConstPtr local_pos =
      ros::topic::waitForMessage<geometry_msgs::PointStamped>(
          "dji_sdk/local_position", env->nh_,
          ros::Duration(env->message_timeout_s_));
  if (local_pos == nullptr) {
    ROS_WARN("Could not receive current x, y");
    return false;
  }

  *x = local_pos->point.x;
  *y = local_pos->point.y;

  return true;
}

bool Waypoint::getTrajectoryStartXY(double* x, double* y) {
  ROS_ASSERT(x);
  ROS_ASSERT(y);

  if (!env->trajectory_start_) {
    return false;
  }

  *x = env->trajectory_start_.value().x();
  *y = env->trajectory_start_.value().y();

  return true;
}

bool Waypoint::getTrajectoryStartXYZ(double* x, double* y, double* z) {
  ROS_ASSERT(x);
  ROS_ASSERT(y);

  if (!env->trajectory_start_) {
    return false;
  }

  getTrajectoryStartXY(x, y);
  *z = env->trajectory_start_.value().z();

  return true;
}

bool Waypoint::getTakeoffXY(double* x, double* y) {
  ROS_ASSERT(x);
  ROS_ASSERT(y);

  if (!env->home_) {
    return false;
  }

  *x = env->home_.value().x();
  *y = env->home_.value().y();

  return true;
}

bool Waypoint::getCurrentYaw(double* yaw) {
  ROS_ASSERT(yaw == nullptr);

  // Receive current x,y.
  geometry_msgs::QuaternionStampedConstPtr attitude =
      ros::topic::waitForMessage<geometry_msgs::QuaternionStamped>(
          "dji_sdk/attitude", env->nh_, ros::Duration(env->message_timeout_s_));
  if (attitude == nullptr) {
    ROS_WARN("Could not receive current yaw");
    return false;
  }

  Eigen::Quaterniond q;
  tf::quaternionMsgToEigen(attitude->quaternion, q);
  *yaw = mav_msgs::yawFromQuaternion(q);

  return true;
}

const bool kIsPeriodic = true;
WaypointFactory::WaypointFactory(tml::TaskEnvironmentPtr env)
    : Parent("Waypoint", "Fly to a predefined waypoint.", kIsPeriodic, env) {}

}  // namespace tasks
}  // namespace fm_task_manager

using namespace task_manager_lib;
DYNAMIC_TASK(fm_task_manager::tasks::WaypointFactory);
