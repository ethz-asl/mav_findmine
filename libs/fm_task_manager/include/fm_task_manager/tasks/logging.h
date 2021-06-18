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

#ifndef FM_TASK_MANAGER_TASKS_LOGGING_H_
#define FM_TASK_MANAGER_TASKS_LOGGING_H_

#include <ros/ros.h>
#include <rosbag/recorder.h>
#include <task_manager_lib/TaskDefinition.h>

#include "fm_task_manager/LoggingConfig.h"
#include "fm_task_manager/task_env.h"

namespace tml = task_manager_lib;

namespace fm_task_manager {
namespace tasks {

// Task wrapper to control rosbag logging.
// 1. Launch logging script. (Iterate)
// 2. Check that DJI is in air. (Iterate)
// 3. Stop logging script. (Terminate)
// This is a periodic background task that runs concurrently to all flight
// actions.
class Logging : public tml::TaskInstance<LoggingConfig, TaskEnv> {
 public:
  Logging(tml::TaskDefinitionPtr def, tml::TaskEnvironmentPtr env);

 private:
  tml::TaskIndicator initialise() override;
  tml::TaskIndicator iterate() override;
  tml::TaskIndicator terminate() override;

  bool taken_off_ = false;
};

class LoggingFactory
    : public tml::TaskDefinition<LoggingConfig, TaskEnv, Logging> {
 public:
  LoggingFactory(tml::TaskEnvironmentPtr env);
};

}  // namespace tasks
}  // namespace fm_task_manager

#endif  // FM_TASK_MANAGER_TASKS_LOGGING_H_
