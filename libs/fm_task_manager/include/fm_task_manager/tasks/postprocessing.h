/*
MIT License

Copyright (c) 2021 Rik Baehnemann, ASL, ETH Zurich, Switzerland

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

#ifndef FM_TASK_MANAGER_TASKS_POSTPROCESSING_H_
#define FM_TASK_MANAGER_TASKS_POSTPROCESSING_H_

#include <string>

#include <mav_state_estimation/BatchStatus.h>
#include <ros/ros.h>
#include <task_manager_lib/TaskDefinition.h>

#include "fm_task_manager/PostprocessingConfig.h"
#include "fm_task_manager/task_env.h"

namespace tml = task_manager_lib;

namespace fm_task_manager {
namespace tasks {

// Task wrapper to run and monitor state estimation process
// 1. Call batch processing service. (Initialize)
// 2. Monitor batch processing status. (Iterate)
// 3. Export external sensor positions. (Terminate)
class Postprocessing : public tml::TaskInstance<PostprocessingConfig, TaskEnv> {
 public:
  Postprocessing(tml::TaskDefinitionPtr def, tml::TaskEnvironmentPtr env);

 private:
  tml::TaskIndicator initialise() override;
  tml::TaskIndicator iterate() override;

  std::string batch_bag_;
  void subscribeTopics();
  void receiveBatchStatus(const mav_state_estimation::BatchStatusConstPtr& msg);
  void receiveExportStatus(
      const mav_state_estimation::BatchStatusConstPtr& msg);
  ros::Subscriber batch_status_sub_;
  mav_state_estimation::BatchStatusConstPtr batch_status_ = nullptr;
  ros::Subscriber export_status_sub_;
  mav_state_estimation::BatchStatusConstPtr export_status_ = nullptr;

  enum class InternalState { kBatchProcessing, kExporting };
  InternalState internal_state_ = InternalState::kBatchProcessing;
};

class PostprocessingFactory
    : public tml::TaskDefinition<PostprocessingConfig, TaskEnv,
                                 Postprocessing> {
 public:
  PostprocessingFactory(tml::TaskEnvironmentPtr env);
};

}  // namespace tasks
}  // namespace fm_task_manager

#endif  // FM_TASK_MANAGER_TASKS_POSTPROCESSING_H_
