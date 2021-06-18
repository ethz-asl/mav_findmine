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

#ifndef FM_TRAJECTORIES_RECTANGLE_TRAJECTORY_H_
#define FM_TRAJECTORIES_RECTANGLE_TRAJECTORY_H_

#include <mav_trajectory_generation_ros/input_constraints.h>
#include <Eigen/Core>
#include "fm_trajectories/polygon_trajectory.h"

namespace fm_trajectories {

namespace mtg = mav_trajectory_generation;

class RectangleTrajectory : public PolygonTrajectory {
 public:
  struct Settings : public PolygonTrajectory::Settings {
    Settings(double A_east, double A_north, double delta_deg, double a,
             double b, double h_A_abs, double h_B_rel, double h_C_rel,
             double h_D_rel, double velocity, double offset_heading_deg,
             const std::weak_ptr<BaseTrajectory>& prev_trajectory,
             const mtg::InputConstraints& input_constraints,
             size_t start_vertex, size_t num_edges)
        : PolygonTrajectory::Settings(
              computeVerticesFromSettings(A_east, A_north, delta_deg, a, b,
                                          h_A_abs, h_B_rel, h_C_rel, h_D_rel),
              velocity, offset_heading_deg, prev_trajectory, input_constraints,
              start_vertex, num_edges),
          A_east(A_east),
          A_north(A_north),
          delta_deg(delta_deg),
          a(a),
          b(b),
          h_A_abs(h_A_abs),
          h_B_rel(h_B_rel),
          h_C_rel(h_C_rel),
          h_D_rel(h_D_rel) {}
    double A_east;
    double A_north;
    double delta_deg;
    double a;
    double b;
    double h_A_abs;  // Absolute altitude A.
    double h_B_rel;  // Relative altitude B.
    double h_C_rel;  // Relative altitude C.
    double h_D_rel;  // Relative altitude D.

   private:
    std::vector<Eigen::Vector3d> computeVerticesFromSettings(
        double A_east, double A_north, double delta_deg, double a, double b,
        double h_A_abs, double h_B_rel, double h_C_rel, double h_D_rel) const;
  };

  // Rectangle, defined by its 2 edges a and b, heading delta of vertex a,
  // heading counted from east counter clockwise, east being 0 degrees, north
  // being 90 degrees. Point A given by geographic coordinates. The rectangle is
  // complemented by its corners B, C, D in counterclockwise direction starting
  // from A to B to C to D to A. Each corner has its individual relative height,
  // i.e., h(A), h(B), h(C), h(D).
  RectangleTrajectory(
      const std::shared_ptr<RectangleTrajectory::Settings>& settings)
      : PolygonTrajectory(settings) {}

  bool toYaml(YAML::Node* node) override;
};

}  // namespace fm_trajectories

#endif  // FM_TRAJECTORIES_RECTANGLE_TRAJECTORY_H_
