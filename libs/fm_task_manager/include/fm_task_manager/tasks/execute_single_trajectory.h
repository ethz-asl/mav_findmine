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

#ifndef FM_TASK_MANAGER_TASKS_EXECUTE_SINGLE_TRAJECTORY_H_
#define FM_TASK_MANAGER_TASKS_EXECUTE_SINGLE_TRAJECTORY_H_

#include <limits>

#include <geometry_msgs/Vector3Stamped.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <task_manager_lib/TaskDefinition.h>
#include <Eigen/Core>

#include "fm_task_manager/ExecuteSingleTrajectoryConfig.h"
#include "fm_task_manager/task_env.h"

namespace tml = task_manager_lib;

namespace fm_task_manager {
namespace tasks {

// Task to execute a precomputed trajectory.
// 1. Obtain trajectory. (Init)
// 2. Update goal information. (Init)
// 3. Obtain control authority. (Init)
// 4. Send trajectory to controller. (Init)
// 5. Check finished. (Iterate)
// 6. Release control authority. (Terminate)
class ExecuteSingleTrajectory
    : public tml::TaskInstance<ExecuteSingleTrajectoryConfig, TaskEnv> {
 public:
  ExecuteSingleTrajectory(tml::TaskDefinitionPtr def,
                          tml::TaskEnvironmentPtr env);

 private:
  tml::TaskIndicator initialise() override;
  tml::TaskIndicator iterate() override;
  tml::TaskIndicator terminate() override;

  void subscribeTopics();
  ros::Subscriber local_position_sub_;
  ros::Subscriber position_error_sub_;
  ros::Subscriber ctrl_auth_sub_;
  void receiveVelocity(const geometry_msgs::Vector3StampedConstPtr& msg);
  void receivePositionError(const std_msgs::Float32ConstPtr& msg);
  void receiveCtrlAuthority(const std_msgs::BoolConstPtr& msg);

  // States.
  ros::Time trajectory_start_time_;
  ros::Duration trajectory_duration_;
  double tracking_error_m_ = std::numeric_limits<double>::max();
  double abs_velocity_ = std::numeric_limits<double>::max();
  bool has_ctrl_ = false;
};

class ExecuteSingleTrajectoryFactory
    : public tml::TaskDefinition<ExecuteSingleTrajectoryConfig, TaskEnv,
                                 ExecuteSingleTrajectory> {
 public:
  ExecuteSingleTrajectoryFactory(tml::TaskEnvironmentPtr env);
};

}  // namespace tasks
}  // namespace fm_task_manager

#endif  // FM_TASK_MANAGER_TASKS_EXECUTE_SINGLE_TRAJECTORY_H_
