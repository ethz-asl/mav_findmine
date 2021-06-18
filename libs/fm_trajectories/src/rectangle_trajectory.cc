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

#include "fm_trajectories/rectangle_trajectory.h"

#include <angles/angles.h>
#include <Eigen/Geometry>
#include <cmath>

namespace fm_trajectories {

std::vector<Eigen::Vector3d>
RectangleTrajectory::Settings::computeVerticesFromSettings(
    double A_east, double A_north, double delta_deg, double a, double b,
    double h_A_abs, double h_B_rel, double h_C_rel, double h_D_rel) const {
  // Express rectangle edges without changed heading.
  // Make height absolute.
  h_B_rel += h_A_abs;
  h_C_rel += h_A_abs;
  h_D_rel += h_A_abs;
  Eigen::Vector3d v_a(a, 0.0, h_B_rel - h_A_abs);
  Eigen::Vector3d v_b(0.0, b, h_C_rel - h_B_rel);
  Eigen::Vector3d v_c(-a, 0.0, h_D_rel - h_C_rel);

  // Change heading.
  double delta = angles::from_degrees(delta_deg);
  Eigen::Quaterniond q_heading(
      Eigen::AngleAxisd(delta, Eigen::Vector3d::UnitZ()));

  v_a = q_heading * v_a;
  v_b = q_heading * v_b;
  v_c = q_heading * v_c;

  std::vector<Eigen::Vector3d> vertices(4);
  vertices[0] << A_east, A_north, h_A_abs;
  vertices[1] = vertices[0] + v_a;
  vertices[2] = vertices[1] + v_b;
  vertices[3] = vertices[2] + v_c;

  return vertices;
}

bool RectangleTrajectory::toYaml(YAML::Node* node) {
  CHECK_NOTNULL(node);
  *node = YAML::Node();

  Eigen::Vector3d A_wgs84;
  if (!geotf_.convert(
          "enu",
          Eigen::Vector3d(
              std::static_pointer_cast<RectangleTrajectory::Settings>(settings_)
                  ->A_east,
              std::static_pointer_cast<RectangleTrajectory::Settings>(settings_)
                  ->A_north,
              std::static_pointer_cast<RectangleTrajectory::Settings>(settings_)
                  ->h_A_abs),
          "wgs84", &A_wgs84))
    return false;

  (*node)["A_lat"] = A_wgs84.x();
  (*node)["A_lon"] = A_wgs84.y();
  (*node)["delta_deg"] =
      std::static_pointer_cast<RectangleTrajectory::Settings>(settings_)
          ->delta_deg;
  (*node)["a"] =
      std::static_pointer_cast<RectangleTrajectory::Settings>(settings_)->a;
  (*node)["b"] =
      std::static_pointer_cast<RectangleTrajectory::Settings>(settings_)->b;
  (*node)["velocity"] =
      std::static_pointer_cast<RectangleTrajectory::Settings>(settings_)
          ->velocity;
  (*node)["h_A_abs"] =
      std::static_pointer_cast<RectangleTrajectory::Settings>(settings_)
          ->h_A_abs;
  (*node)["h_B_rel"] =
      std::static_pointer_cast<RectangleTrajectory::Settings>(settings_)
          ->h_B_rel;
  (*node)["h_C_rel"] =
      std::static_pointer_cast<RectangleTrajectory::Settings>(settings_)
          ->h_C_rel;
  (*node)["h_D_rel"] =
      std::static_pointer_cast<RectangleTrajectory::Settings>(settings_)
          ->h_D_rel;
  (*node)["offset_heading_deg"] =
      std::static_pointer_cast<RectangleTrajectory::Settings>(settings_)
          ->offset_heading_deg;
  (*node)["start_vertex"] =
      std::static_pointer_cast<RectangleTrajectory::Settings>(settings_)
          ->start_vertex;
  (*node)["num_edges"] =
      std::static_pointer_cast<RectangleTrajectory::Settings>(settings_)
          ->num_edges;
  (*node)["trajectory_type"] = static_cast<int>(TrajectoryType::Rectangle);

  return true;
}

}  // namespace fm_trajectories
