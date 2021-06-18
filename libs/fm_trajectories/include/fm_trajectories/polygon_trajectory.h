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
// Trajectory following polygon vertices. Between the vertices we want constant
// velocity and heading, thus we add extra turning and acceleration maneuveres.
// Motion sequence is:
// Stop->Turn->Accelerate->Constant velocity strip->Stop->....
class PolygonTrajectory : public BaseTrajectory {
 public:
  struct Settings : public BaseTrajectory::Settings {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Settings(const std::vector<Eigen::Vector3d>& vertices, double velocity,
             double offset_heading_deg,
             const std::weak_ptr<BaseTrajectory>& prev_trajectory,
             const mtg::InputConstraints& input_constraints,
             size_t start_vertex, size_t num_edges);
    std::vector<Eigen::Vector3d> vertices;
    struct Edge {
      Edge() : direction(Eigen::Vector3d::Zero()), length(0.0), angle(0.0) {}
      Edge(const Eigen::Vector3d& from, const Eigen::Vector3d& to);
      Eigen::Vector3d direction;
      double length;
      double angle;
    };
    std::vector<Edge> edges;
    size_t start_vertex;
    size_t num_edges;
    bool is_waypoint_list;

   private:
    std::vector<Edge> computeEdges(
        const std::vector<Eigen::Vector3d>& vertices) const;
  };

 public:
  PolygonTrajectory(
      const std::shared_ptr<PolygonTrajectory::Settings>& settings);

  virtual bool toYaml(YAML::Node* node) override;

 protected:
  virtual void samplePositionVertices() override;
  virtual void sampleYawVertices() override;
  virtual void sampleTimes() override;
};

}  // namespace fm_trajectories

#endif  // FM_TRAJECTORIES_POLYGON_TRAJECTORY_H_
