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

#ifndef FM_TRAJECTORIES_TRAJECTORY_GENERATOR_H
#define FM_TRAJECTORIES_TRAJECTORY_GENERATOR_H

#include <fm_comm/TrajectoryService.h>
#include <geotf/geodetic_converter.h>
#include <mav_trajectory_generation/trajectory.h>
#include <mav_trajectory_generation/vertex.h>
#include <ros/ros.h>

#include "fm_trajectories/base_trajectory.h"

namespace fm_trajectories {

namespace mtg = mav_trajectory_generation;

class TrajectoryGenerator {
 public:
  TrajectoryGenerator(const ros::NodeHandle& nh,
                      const ros::NodeHandle& nh_private);

 private:
  void advertiseServices();
  void advertiseTopics();

  // Receive a desired trajectory, transform it to the current ENU frame, plan
  // to the starting point and return to the goal.
  bool generateTrajectory(fm_comm::TrajectoryService::Request& req,
                          fm_comm::TrajectoryService::Response& res);

  // Set vertices from pose and twist. All other derivatives up to the maximum
  // derivative order will be set zero. Angular velocity is transformed from
  // body to world frame.
  void vertexFromPoseTwist(const geometry_msgs::Pose& pose,
                           const geometry_msgs::Twist& twist,
                           mtg::Vertex* pos_vertex,
                           mtg::Vertex* yaw_vertex) const;

  bool geodeticReferenceFromYaml(const YAML::Node& node);

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  ros::ServiceServer trajectory_srv_;

  ros::Publisher trajectory_marker_pub_;

  std::unique_ptr<BaseTrajectory> base_trajectory_;

  geotf::GeodeticConverter geotf_;
};
}  // namespace fm_trajectories

#endif  // FM_TRAJECTORIES_TRAJECTORY_GENERATOR_H
