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

#ifndef FM_MISSION_PLANNER_MISSION_H_
#define FM_MISSION_PLANNER_MISSION_H_
#include <iostream>
#include <memory>
#include <vector>

#include <fm_trajectories/base_trajectory.h>
#include <mav_trajectory_generation_ros/input_constraints.h>
#include <visualization_msgs/MarkerArray.h>

namespace fm_mission_planner {

class Mission {
 public:
  Mission() : reference_set_(false) {}

  void addTrajectory(
      const std::shared_ptr<fm_trajectories::BaseTrajectory>& trajectory);
  void setInputConstraints(
      const mav_trajectory_generation::InputConstraints& input_constraints);
  mav_trajectory_generation::InputConstraints getInputConstraints() const;

  void clear();

  std::string toString() const;
  std::string exportTrajectory() const;

  void setGeodeticReference(double lat, double lon, double alt);
  bool referenceToYaml(YAML::Node* node) const;

  visualization_msgs::MarkerArray getTrajectoryMarkers() const;
  visualization_msgs::MarkerArray getTrajectoryDeleteMarkers() const;

 private:
  std::vector<std::shared_ptr<fm_trajectories::BaseTrajectory>> trajectory_vec_;

  bool reference_set_;
  double lat_;
  double lon_;
  double alt_;
  mav_trajectory_generation::InputConstraints input_constraints_;
};

}  // namespace fm_mission_planner

#endif  // FM_MISSION_PLANNER_MISSION_H_
