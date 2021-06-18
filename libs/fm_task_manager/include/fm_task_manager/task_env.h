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

#ifndef FM_TASK_MANAGER_TASK_ENV_H_
#define FM_TASK_MANAGER_TASK_ENV_H_

#include <mav_trajectory_generation/trajectory.h>
#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <task_manager_lib/TaskDefinition.h>
#include <optional>

namespace fm_task_manager {

// The task environment containing common subscribers, methods, and states.
class TaskEnv : public task_manager_lib::TaskEnvironment {
 public:
  TaskEnv(ros::NodeHandle& nh, ros::NodeHandle& nh_private);

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  std_msgs::UInt8 display_mode_;
  std_msgs::UInt8 flight_status_;

  bool obtainCtrlAuthority();
  bool releaseCtrlAuthority();

  double message_timeout_s_;  // Time to wait for a topic to arrive.
  ros::Publisher trajectory_pub_;
  mav_trajectory_generation::Trajectory trajectory_;

  std::optional<Eigen::Vector3d> trajectory_start_;
  std::optional<Eigen::Vector3d> home_;

 private:
  void getRosParameters();
  void advertiseTopics();
  void subscribeTopics();
  void initializeStates();

  void receiveDisplayMode(const std_msgs::UInt8ConstPtr& msg);
  void receiveFlightStatus(const std_msgs::UInt8ConstPtr& msg);

  // Debug info published by callDjiTask in case of failure.
  template <class Service>
  void printResponseWarning(const typename Service::Response& res) const;

  ros::Subscriber display_mode_sub_;
  ros::Subscriber flight_status_sub_;
};

}  // namespace fm_task_manager

#endif  // FM_TASK_MANAGER_TASK_ENV_H_
