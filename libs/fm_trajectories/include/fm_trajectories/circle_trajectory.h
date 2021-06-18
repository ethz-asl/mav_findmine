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

#ifndef FM_TRAJECTORIES_CIRCLE_TRAJECTORY_H_
#define FM_TRAJECTORIES_CIRCLE_TRAJECTORY_H_

#include <mav_trajectory_generation/trajectory.h>
#include <yaml-cpp/yaml.h>
#include <cmath>

#include "fm_trajectories/base_trajectory.h"

namespace fm_trajectories {

namespace mtg = mav_trajectory_generation;

const double kMinArcLength = 0.1;

// Arc, defined by center point (ENU), radius [m], start heading with 0 degree
// being east and arc length as angular value in degrees. If arc velocity is
// negative, fly clockwise. The arc is flown in a horizontal plane above home
// point.
class CircleTrajectory : public BaseTrajectory {
 public:
  struct Settings : public BaseTrajectory::Settings {
    Settings(double center_east, double center_north, double radius,
             double start_heading_deg, double arc_length_deg_in,
             double altitude, double velocity, double offset_heading_deg,
             const std::weak_ptr<BaseTrajectory>& prev_trajectory,
             const mtg::InputConstraints& input_constraints,
             double circle_deviation_ratio = 0.01);
    double center_east;
    double center_north;
    double radius;
    double start_heading_deg;
    double arc_length_deg;
    double altitude;
    double circle_deviation_ratio;  // Deviation ratio of circle from polygon.

    double computeAngularVelocity() const;
    double computeAcceleration() const;
  };
  CircleTrajectory(const std::shared_ptr<CircleTrajectory::Settings>& settings);

  bool toYaml(YAML::Node* node) override;

 protected:
  void samplePositionVertices() override;
  void sampleYawVertices() override;
  void sampleTimes() override;

 private:
  double computeDeltaAngle(double angular_velocity) const;

  double start_rad_;
  double arc_length_rad_;
  int num_vertices_;
};

}  // namespace fm_trajectories

#endif  // FM_TRAJECTORIES_CIRCLE_TRAJECTORY_H_
