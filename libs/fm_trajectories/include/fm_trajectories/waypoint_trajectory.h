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

#ifndef FM_TRAJECTORIES_POLYGON_TRAJECTORY_H_
#define FM_TRAJECTORIES_POLYGON_TRAJECTORY_H_

#include <mav_trajectory_generation_ros/feasibility_recursive.h>
#include <mav_trajectory_generation_ros/input_constraints.h>
#include <Eigen/Core>
#include "fm_trajectories/base_trajectory.h"

namespace fm_trajectories {
// Trajectory following waypoints. For now we infer a velocity ramp profile
// between the trajectories and do not change the heading.
// Stop->Waypoint->Stop->Waypoint...
class WaypointTrajectory : public BaseTrajectory {
 public:
  enum VertexType { kTurn = 0, kVelStart, kVelEnd, kStop };

  struct Settings : public BaseTrajectory::Settings {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Settings(const std::vector<Eigen::Vector3d>& vertices_in, double velocity,
             double offset_heading_deg,
             const std::weak_ptr<BaseTrajectory>& prev_trajectory,
             const mtg::InputConstraints& input_constraints);
    std::vector<Eigen::Vector3d> vertices;
    struct Edge {
      Edge() : direction(Eigen::Vector3d::Zero()), length(0.0), angle(0.0) {}
      Edge(const Eigen::Vector3d& from, const Eigen::Vector3d& to);
      Eigen::Vector3d direction;
      double length;
      double angle;
    };
    std::vector<Edge> edges;

   private:
    std::vector<Edge> computeEdges(
        const std::vector<Eigen::Vector3d>& vertices) const;
  };

 public:
  WaypointTrajectory(
      const std::shared_ptr<WaypointTrajectory::Settings>& settings);

  virtual bool toYaml(YAML::Node* node) override;

 protected:
  virtual void samplePositionVertices() override;
  virtual void sampleYawVertices() override;
  virtual void sampleTimes() override;

 private:
  bool computeAccelerationDistance(const Eigen::Vector3d& from,
                                   const Eigen::Vector3d& to, double v_max,
                                   double a_max, double* acc_distance);

  std::vector<VertexType> vertex_types_;
  std::map<size_t, size_t> map_pos_v_to_edge_;
};

}  // namespace fm_trajectories

#endif  // FM_TRAJECTORIES_POLYGON_TRAJECTORY_H_
