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

#ifndef FM_TASK_MANAGER_TASKS_CAMERA_H_
#define FM_TASK_MANAGER_TASKS_CAMERA_H_

#include <geometry_msgs/PointStamped.h>
#include <ros/ros.h>
#include <task_manager_lib/TaskDefinition.h>
#include <Eigen/Core>

#include "fm_task_manager/CameraConfig.h"
#include "fm_task_manager/task_env.h"

namespace tml = task_manager_lib;

namespace fm_task_manager {
namespace tasks {

// Task wrapper to control the camera.
// 1. Move gimbal into configured pitch position. (Iterate)
// 2. Take a picture according to FOV and overlap. (Iterate)
// 3. Reset gimbal position. (Terminate)
// Only terminates on error or preemption.
class Camera : public tml::TaskInstance<CameraConfig, TaskEnv> {
 public:
  Camera(tml::TaskDefinitionPtr def, tml::TaskEnvironmentPtr env);

 private:
  tml::TaskIndicator initialise() override;
  tml::TaskIndicator iterate() override;
  tml::TaskIndicator terminate() override;

  void advertiseTopics();
  ros::Publisher gimbal_angle_cmd_pub_;
  void setGimbalPitch();

  void subscribeServices();
  ros::ServiceClient picture_client_;
  bool takePicture();

  void subscribeTopics();
  ros::Subscriber local_position_sub_;
  void receiveLocalPosition(const geometry_msgs::PointStampedConstPtr& msg);

  double computePictureDistance() const;

  // States
  double pitch_cmd_;
  bool has_prev_position_;
  double distance_since_picture_;
  Eigen::Vector3d prev_position_;
};

class CameraFactory
    : public tml::TaskDefinition<CameraConfig, TaskEnv, Camera> {
 public:
  CameraFactory(tml::TaskEnvironmentPtr env);
};

}  // namespace tasks
}  // namespace fm_task_manager

#endif  // FM_TASK_MANAGER_TASKS_CAMERA_H_
