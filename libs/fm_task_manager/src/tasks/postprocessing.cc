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

#include "fm_task_manager/tasks/postprocessing.h"

#include <mav_state_estimation/BagToCsv.h>
#include <mav_state_estimation/Batch.h>
#include <task_manager_msgs/TaskStatus.h>

namespace tmm = task_manager_msgs;

namespace fm_task_manager {
namespace tasks {

Postprocessing::Postprocessing(tml::TaskDefinitionPtr def,
                               tml::TaskEnvironmentPtr env)
    : Parent(def, env) {}

const bool kQueueSize = 1;
void Postprocessing::subscribeTopics() {
  batch_status_sub_ =
      env->nh_.subscribe("mav_state_estimator/batch_status", kQueueSize,
                         &Postprocessing::receiveBatchStatus, this);
  export_status_sub_ =
      env->nh_.subscribe("bag_to_csv/status", kQueueSize,
                         &Postprocessing::receiveExportStatus, this);
}

tml::TaskIndicator Postprocessing::initialise() {
  batch_status_ = nullptr;
  subscribeTopics();

  batch_bag_ = cfg.directory;
  batch_bag_ += "/batch.bag";

  // State estimator batch computation.
  ROS_INFO_STREAM("Calling batch computation with target " << batch_bag_);
  auto batch_client = env->nh_.serviceClient<mav_state_estimation::Batch>(
      "mav_state_estimator/compute_batch_solution");
  mav_state_estimation::Batch batch_req;
  batch_req.request.bag_file = batch_bag_;

  if (!batch_client.call(batch_req)) {
    ROS_WARN("Cannot engage batch computation.");
    return tmm::TaskStatus::TASK_INITIALISATION_FAILED;
  }

  internal_state_ = InternalState::kBatchProcessing;
  return tmm::TaskStatus::TASK_INITIALISED;
}

void Postprocessing::receiveBatchStatus(
    const mav_state_estimation::BatchStatusConstPtr& msg) {
  ROS_INFO("Processed %lu out of %lu messages.", msg->current_idx,
           msg->total_idx);
  batch_status_ = msg;
}

void Postprocessing::receiveExportStatus(
    const mav_state_estimation::BatchStatusConstPtr& msg) {
  ROS_INFO("Exported %lu out of %lu messages.", msg->current_idx,
           msg->total_idx);
  export_status_ = msg;
}

tml::TaskIndicator Postprocessing::iterate() {
  if (internal_state_ == InternalState::kBatchProcessing && batch_status_ &&
      batch_status_->finished) {
    ROS_INFO("Finished batch processing.");

    // Call export
    auto csv_client = env->nh_.serviceClient<mav_state_estimation::BagToCsv>(
        "bag_to_csv/bag_to_csv");

    mav_state_estimation::BagToCsv csv_req;
    csv_req.request.bag_file = batch_bag_;
    csv_req.request.topics = cfg.topics;
    ROS_INFO("Calling %s with bag %s and topics %s.",
             csv_client.getService().c_str(), csv_req.request.bag_file.c_str(),
             csv_req.request.topics.c_str());
    if (!csv_client.call(csv_req)) {
      ROS_ERROR("Failed to start CSV export.");
      return tmm::TaskStatus::TASK_FAILED;
    }
    internal_state_ = InternalState::kExporting;
  } else if (internal_state_ == InternalState::kExporting && export_status_ &&
             export_status_->finished) {
    ROS_INFO("Finished exporting.");
    return tmm::TaskStatus::TASK_COMPLETED;
  }

  return tmm::TaskStatus::TASK_RUNNING;
}

const bool kIsPeriodic = true;
PostprocessingFactory::PostprocessingFactory(tml::TaskEnvironmentPtr env)
    : Parent("Postprocessing", "Execute batch postprocessing.", kIsPeriodic,
             env) {}

}  // namespace tasks
}  // namespace fm_task_manager

using namespace task_manager_lib;
DYNAMIC_TASK(fm_task_manager::tasks::PostprocessingFactory);
