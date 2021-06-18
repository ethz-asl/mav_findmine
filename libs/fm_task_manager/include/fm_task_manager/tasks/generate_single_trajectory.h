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

#ifndef FM_TASK_MANAGER_TASKS_GENERATE_SINGLE_TRAJECTORY_H_
#define FM_TASK_MANAGER_TASKS_GENERATE_SINGLE_TRAJECTORY_H_

#include <fm_comm/TrajectoryService.h>

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <task_manager_lib/TaskDefinition.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include "fm_task_manager/GenerateSingleTrajectoryConfig.h"
#include "fm_task_manager/task_env.h"

namespace tml = task_manager_lib;

namespace fm_task_manager {
namespace tasks {

// Generate a single measurement trajectory.
// 1. On changing parameters recalculate trajectory. (Iterate)
// 2. On approval terminate. (Iterate)
class GenerateSingleTrajectory
    : public tml::TaskInstance<GenerateSingleTrajectoryConfig, TaskEnv> {
 public:
  GenerateSingleTrajectory(tml::TaskDefinitionPtr def,
                           tml::TaskEnvironmentPtr env);

 private:
  tml::TaskIndicator initialise() override;
  tml::TaskIndicator iterate() override;

  bool approveTrajectory(std_srvs::Empty::Request& req,
                         std_srvs::Empty::Response& res);

  bool setLocalPosRef();
  bool createTrajectory();

  // Services.
  ros::ServiceServer approval_srv_;

  // States.;
  bool trajectory_approved_;
  bool trajectory_generated_;

  tf2_ros::StaticTransformBroadcaster tf_br_;
};

class GenerateSingleTrajectoryFactory
    : public tml::TaskDefinition<GenerateSingleTrajectoryConfig, TaskEnv,
                                 GenerateSingleTrajectory> {
 public:
  GenerateSingleTrajectoryFactory(tml::TaskEnvironmentPtr env);
};

}  // namespace tasks
}  // namespace fm_task_manager

#endif  // FM_TASK_MANAGER_TASKS_GENERATE_SINGLE_TRAJECTORY_H_
