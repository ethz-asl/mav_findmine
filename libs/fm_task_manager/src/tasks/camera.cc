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

#include "fm_task_manager/tasks/camera.h"

#include <cmath>
#include <limits>

#include <dji_sdk/CameraAction.h>
#include <dji_sdk/Gimbal.h>
#include <eigen_conversions/eigen_msg.h>
#include <task_manager_msgs/TaskStatus.h>

#include "fm_comm/dji_interface.h"

namespace tmm = task_manager_msgs;

namespace fm_task_manager {
namespace tasks {

Camera::Camera(tml::TaskDefinitionPtr def, tml::TaskEnvironmentPtr env)
    : Parent(def, env) {}

const size_t kQueueSize = 1;
void Camera::advertiseTopics() {
  const bool kLatchTopic = true;
  gimbal_angle_cmd_pub_ = env->nh_.advertise<dji_sdk::Gimbal>(
      "dji_sdk/gimbal_angle_cmd", kQueueSize, kLatchTopic);
}

void Camera::setGimbalPitch() {
  const bool kIsAbsolute = true;
  const bool kIgnoreYaw = false;
  const bool kIgnoreRoll = false;
  const bool kIgnorePitch = false;

  dji_sdk::Gimbal gimbal;
  gimbal.header.stamp = ros::Time::now();
  gimbal.header.frame_id = "base";

  gimbal.mode = 0;
  gimbal.mode |= kIsAbsolute << 0;
  gimbal.mode |= kIgnoreYaw << 1;
  gimbal.mode |= kIgnoreRoll << 2;
  gimbal.mode |= kIgnorePitch << 3;
  gimbal.ts = cfg.gimbal_time;
  gimbal.roll = 0.0;
  gimbal.pitch = pitch_cmd_;
  gimbal.yaw = 0.0;

  ROS_INFO_STREAM("Setting new gimbal position with pitch: " << pitch_cmd_);
  gimbal_angle_cmd_pub_.publish(gimbal);

  // Give time for gimbal to reposition.
  ros::Duration(cfg.gimbal_time).sleep();
}

void Camera::subscribeServices() {
  picture_client_ =
      env->nh_.serviceClient<dji_sdk::CameraAction>("dji_sdk/camera_action");
}

bool Camera::takePicture() {
  dji_sdk::CameraAction cam_action;
  cam_action.request.camera_action =
      dji_sdk::CameraAction::Request::CAMERA_ACTION_TAKE_PICTURE;
  if (!picture_client_.call(cam_action)) {
    ROS_WARN("Cannot call DJI camera action service.");
    return false;
  }

  if (!cam_action.response.result) {
    ROS_WARN("Camera trigger not successful.");
    return false;
  }

  distance_since_picture_ = 0.0;
  return true;
}

void Camera::subscribeTopics() {
  local_position_sub_ = env->nh_.subscribe("dji_sdk/local_position", kQueueSize,
                                           &Camera::receiveLocalPosition, this);
}

double Camera::computePictureDistance() const {
  return (1 - cfg.longitudinal_overlap) * 2 * cfg.altitude *
         std::tan(cfg.longitudinal_fov / 2.0);
}

void Camera::receiveLocalPosition(
    const geometry_msgs::PointStampedConstPtr& msg) {
  // Update distance travelled.
  Eigen::Vector3d curr_position;
  tf::pointMsgToEigen(msg->point, curr_position);
  if (has_prev_position_) {
    distance_since_picture_ += (curr_position - prev_position_).norm();
  }
  prev_position_ = curr_position;
  has_prev_position_ = true;
}

tml::TaskIndicator Camera::initialise() {
  // Reset states.
  pitch_cmd_ = std::numeric_limits<double>::max();
  has_prev_position_ = false;
  distance_since_picture_ = computePictureDistance();
  prev_position_ = Eigen::Vector3d::Zero();

  // ROS.
  advertiseTopics();
  subscribeTopics();
  subscribeServices();
  ros::spinOnce();

  return tmm::TaskStatus::TASK_INITIALISED;
}

tml::TaskIndicator Camera::iterate() {
  // Update gimbal position.
  if (pitch_cmd_ != cfg.pitch_cmd) {
    pitch_cmd_ = cfg.pitch_cmd;
    setGimbalPitch();
    // Immediately take picture after succesful reset.
    distance_since_picture_ = computePictureDistance();
  }

  // Shoot picture.
  if (distance_since_picture_ >= computePictureDistance()) {
    if (!takePicture()) {
      return tmm::TaskStatus::TASK_FAILED;
    }
  }

  return tmm::TaskStatus::TASK_RUNNING;
}

tml::TaskIndicator Camera::terminate() {
  // One last picture.
  if (!takePicture()) {
    return tmm::TaskStatus::TASK_FAILED;
  }

  // Reset gimbal to default position.
  pitch_cmd_ = 0.0;
  setGimbalPitch();

  return tmm::TaskStatus::TASK_TERMINATED;
}

const bool kIsPeriodic = true;
CameraFactory::CameraFactory(tml::TaskEnvironmentPtr env)
    : Parent("Camera", "Orient gimbal and shoot photos.", kIsPeriodic, env) {}

}  // namespace tasks
}  // namespace fm_task_manager

using namespace task_manager_lib;
DYNAMIC_TASK(fm_task_manager::tasks::CameraFactory);
