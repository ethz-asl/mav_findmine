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

#include "fm_task_manager/tasks/generate_single_trajectory.h"

#include <dji_sdk/SetLocalPosRef.h>
#include <eigen_conversions/eigen_msg.h>
#include <fm_comm/TrajectoryService.h>
#include <fm_comm/dji_interface.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>
#include <sensor_msgs/NavSatFix.h>
#include <task_manager_msgs/TaskStatus.h>

namespace tmm = task_manager_msgs;
namespace mtg = mav_trajectory_generation;

namespace fm_task_manager {
namespace tasks {

GenerateSingleTrajectory::GenerateSingleTrajectory(tml::TaskDefinitionPtr def,
                                                   tml::TaskEnvironmentPtr env)
    : Parent(def, env) {}

tml::TaskIndicator GenerateSingleTrajectory::initialise() {
  // States:
  trajectory_approved_ = false;
  trajectory_generated_ = false;
  env->trajectory_.clear();

  // Services:
  approval_srv_ = env->nh_private_.advertiseService(
      "approve_trajectory", &GenerateSingleTrajectory::approveTrajectory, this);

  return tmm::TaskStatus::TASK_INITIALISED;
}

tml::TaskIndicator GenerateSingleTrajectory::iterate() {
  if (!(trajectory_generated_ = createTrajectory())) {
    ROS_WARN_THROTTLE(1.0, "Cannot create trajectory.");
  }

  if (trajectory_approved_) {
    if (!trajectory_generated_) {
      ROS_WARN("Trajectory approved, but no trajectory generated.");
      trajectory_approved_ = false;
      return tmm::TaskStatus::TASK_RUNNING;
    }

    ROS_INFO("Trajectory approved.");
    return tmm::TaskStatus::TASK_COMPLETED;
  }

  return tmm::TaskStatus::TASK_RUNNING;
}

bool GenerateSingleTrajectory::approveTrajectory(
    std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
  trajectory_approved_ = true;
  return true;
}

bool GenerateSingleTrajectory::setLocalPosRef() {
  ros::ServiceClient local_pos_ref_client =
      env->nh_.serviceClient<dji_sdk::SetLocalPosRef>(
          "dji_sdk/set_local_pos_ref");

  dji_sdk::SetLocalPosRef slpr_srv;
  if (!local_pos_ref_client.call(slpr_srv)) {
    ROS_WARN("Cannot call local position reference service.");
    return false;
  }

  if (!slpr_srv.response.result) {
    ROS_WARN("Cannot set local position reference.");
    return false;
  }

  return true;
}

// This function takes care of replanning the preplanned trajectory to address
// DJI GNSS positioning bias. We assume RTK GNSS position, DJI GNSS position,
// and the DJI local reference frame are all placed in the same physical
// location.
// 1. Set the DJI local reference frame to the current DJI GNSS position.
// 2. Create trajectory relative to RTK GNSS position.
bool GenerateSingleTrajectory::createTrajectory() {
  fm_comm::TrajectoryService srv;

  // Set local reference frame
  if (!setLocalPosRef()) {
    return false;
  }

  // Get current state for start/goal pose.
  mav_msgs::EigenMavState mav_state_ENU;
  if (!fm_comm::queryStateENU(&env->nh_, &mav_state_ENU)) {
    ROS_ERROR("Could not receive current state.");
    return false;
  }
  // Set altitude to desired altitude.
  mav_state_ENU.position_W.z() = cfg.altitude;

  // Start and goal pose identical, zero twist.
  tf::pointEigenToMsg(mav_state_ENU.position_W,
                      srv.request.start_pose.position);
  tf::quaternionEigenToMsg(mav_state_ENU.orientation_W_B,
                           srv.request.start_pose.orientation);
  srv.request.goal_pose = srv.request.start_pose;

  // The trajectory should be planned in ENU frame relative to where the UAV is
  // at the moment. Get the current RTK position.

  // Get the current LLH position.
  sensor_msgs::NavSatFixConstPtr rtk_pos =
      ros::topic::waitForMessage<sensor_msgs::NavSatFix>(
          "piksi/position_receiver_0/ros/navsatfix", env->nh_,
          ros::Duration(env->message_timeout_s_));
  if (rtk_pos == nullptr) {
    ROS_WARN("Did not receive current DJI GNSS position.");
    return false;
  }

  // Get the current ENU position.
  geometry_msgs::PointStampedConstPtr enu_pos =
      ros::topic::waitForMessage<geometry_msgs::PointStamped>(
          "piksi/position_receiver_0/ros/pos_enu", env->nh_,
          ros::Duration(env->message_timeout_s_));
  if (enu_pos == nullptr) {
    ROS_WARN("Did not receive current ENU position.");
    return false;
  }

  // Check if RTK fix.
  if (rtk_pos->status.status != sensor_msgs::NavSatStatus::STATUS_GBAS_FIX) {
    ROS_INFO_THROTTLE(1.0, "No RTK fix.");
    return false;
  }

  // Mark this position in TF tree. /ground_ENU_rtk
  // TODO(rikba): Make sure LLH and ENU position are corresponding...
  geometry_msgs::TransformStamped tf;
  tf.header.stamp = enu_pos->header.stamp;
  tf.header.frame_id = "enu";
  tf.child_frame_id = "ground_ENU_rtk";
  tf.transform.translation.x = enu_pos->point.x;
  tf.transform.translation.y = enu_pos->point.y;
  tf.transform.translation.z = enu_pos->point.z;
  tf.transform.rotation.w = 1.0;
  tf_br_.sendTransform(tf);

  // Set relative trajectory position.
  srv.request.target_lat_ENU.data = rtk_pos->latitude;
  srv.request.target_lon_ENU.data = rtk_pos->longitude;
  srv.request.target_alt_ENU.data =
      rtk_pos->altitude;  // This field is ignored.

  // Trajectory file.
  srv.request.mission_file.data = cfg.mission_file;

  // Planning settings.
  srv.request.plan_from_home.data = cfg.plan_from_home;

  ROS_INFO_STREAM("Setting up trajectory from file: "
                  << cfg.mission_file
                  << "\ntarget lat [deg]: " << srv.request.target_lat_ENU.data
                  << ", lon [deg]: " << srv.request.target_lon_ENU.data
                  << ", alt [m]: " << srv.request.target_alt_ENU.data
                  << ", start alt [m]: " << cfg.altitude << ".");

  ros::ServiceClient trajectory_client =
      env->nh_.serviceClient<fm_comm::TrajectoryService>(
          "trajectory_generator/generate_trajectory");

  if (!trajectory_client.call(srv)) {
    ROS_ERROR("Cannot create trajectory.");
    return false;
  }

  // Store trajectory.
  polynomialTrajectoryMsgToTrajectory(srv.response.trajectory,
                                      &env->trajectory_);

  // Store trajectory start and home point
  Eigen::Vector3d trajectory_start;
  tf::pointMsgToEigen(srv.response.start_pos, trajectory_start);
  env->trajectory_start_ = trajectory_start;
  env->home_ = mav_state_ENU.position_W;

  ROS_INFO_STREAM("Trajectory time: " << env->trajectory_.getMaxTime());

  return true;
}

const bool kIsPeriodic = true;
GenerateSingleTrajectoryFactory::GenerateSingleTrajectoryFactory(
    tml::TaskEnvironmentPtr env)
    : Parent("GenerateSingleTrajectory",
             "Generate single measurement trajectories.", kIsPeriodic, env) {}

}  // namespace tasks
}  // namespace fm_task_manager

using namespace task_manager_lib;
DYNAMIC_TASK(fm_task_manager::tasks::GenerateSingleTrajectoryFactory);
