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

#ifndef FM_TRAJECTORIES_STRIP_TRAJECTORY_H_
#define FM_TRAJECTORIES_STRIP_TRAJECTORY_H_

#include <mav_trajectory_generation_ros/input_constraints.h>
#include <Eigen/Core>
#include "fm_trajectories/polygon_trajectory.h"

namespace fm_trajectories {

namespace mtg = mav_trajectory_generation;

// TODO(rikba): Make strip less hacky. It should not be derived from polygon.
class StripTrajectory : public PolygonTrajectory {
 public:
  struct Settings : public PolygonTrajectory::Settings {
    Settings(double A_east, double A_north, double delta_deg, double a,
             double h_A_abs, double h_B_rel, double offset, double velocity,
             double offset_heading_deg,
             const std::weak_ptr<BaseTrajectory>& prev_trajectory,
             const mtg::InputConstraints& input_constraints,
             size_t start_vertex, size_t num_edges)
        : PolygonTrajectory::Settings(
              computeVerticesFromSettings(A_east, A_north, delta_deg, a,
                                          h_A_abs, h_B_rel, offset),
              std::fabs(velocity), offset_heading_deg, prev_trajectory,
              input_constraints, start_vertex, num_edges),
          A_east(A_east),
          A_north(A_north),
          delta_deg(delta_deg),
          a(a),
          h_A_abs(h_A_abs),
          h_B_rel(h_B_rel),
          offset(offset) {
      // Remove closing edge.
      edges.pop_back();
      is_waypoint_list = true;
    }
    double A_east;
    double A_north;
    double delta_deg;
    double a;
    double h_A_abs;  // Absolute altitude A.
    double h_B_rel;  // Relative altitude B.
    double offset;

   private:
    std::vector<Eigen::Vector3d> computeVerticesFromSettings(
        double A_east, double A_north, double delta_deg, double a,
        double h_A_abs, double h_B_rel, double offset) const;
  };

  // Strip, defined by Vertex A, length a, heading delta and relative height B.
  StripTrajectory(const std::shared_ptr<StripTrajectory::Settings>& settings);

  bool toYaml(YAML::Node* node) override;
};

}  // namespace fm_trajectories

#endif  // FM_TRAJECTORIES_STRIP_TRAJECTORY_H_
