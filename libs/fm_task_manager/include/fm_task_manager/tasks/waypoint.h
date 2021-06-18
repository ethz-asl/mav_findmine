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

#ifndef FM_TASK_MANAGER_TASKS_WAYPOINT_H_
#define FM_TASK_MANAGER_TASKS_WAYPOINT_H_

#include <limits>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

#include <task_manager_lib/TaskDefinition.h>

#include "fm_task_manager/WaypointConfig.h"
#include "fm_task_manager/task_env.h"

namespace tml = task_manager_lib;

namespace fm_task_manager {
namespace tasks {

// Task to go to a predefined waypoint.
// 1. Obtain control authority. (Init)
// 2. Continuously send position commands. (Iterate)
// 3. Release control authority. (Terminate)
class Waypoint : public tml::TaskInstance<WaypointConfig, TaskEnv> {
 public:
  Waypoint(tml::TaskDefinitionPtr def, tml::TaskEnvironmentPtr env);

 private:
  tml::TaskIndicator initialise() override;
  tml::TaskIndicator iterate() override;
  tml::TaskIndicator terminate() override;

  void subscribeTopics();
  ros::Subscriber velocity_sub_;
  ros::Subscriber position_error_sub_;
  ros::Subscriber ctrl_auth_sub_;
  void receiveVelocity(const geometry_msgs::Vector3StampedConstPtr& msg);
  void receivePositionError(const std_msgs::Float32ConstPtr& msg);
  void receiveCtrlAuthority(const std_msgs::BoolConstPtr& msg);

  void advertiseTopics();
  ros::Publisher waypoint_pub_;
  void publishWaypoint();

  bool getCurrentXY(double* x, double* y);
  bool getTrajectoryStartXY(double* x, double* y);
  bool getTrajectoryStartXYZ(double* x, double* y, double* z);
  bool getTakeoffXY(double* x, double* y);
  bool getCurrentYaw(double* yaw);

  // States.
  double tracking_error_m_ = std::numeric_limits<double>::max();
  double abs_velocity_ = std::numeric_limits<double>::max();
  bool has_ctrl_ = false;

  // Waypoint types
  enum GoalType {
    SET_ALL = 0,
    FIXED_YAW,
    CHANGE_ALTITUDE,
    TRAJECTORY_START_XY,
    TRAJECTORY_START_XYZ,
    TAKE_OFF_POINT,
  };
};

class WaypointFactory
    : public tml::TaskDefinition<WaypointConfig, TaskEnv, Waypoint> {
 public:
  WaypointFactory(tml::TaskEnvironmentPtr env);
};

}  // namespace tasks
}  // namespace fm_task_manager

#endif  // FM_TASK_MANAGER_TASKS_WAYPOINT_H_
