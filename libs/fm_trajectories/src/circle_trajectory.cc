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

#include "fm_trajectories/circle_trajectory.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include <angles/angles.h>
#include <mav_trajectory_generation/motion_defines.h>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <Eigen/Core>

namespace fm_trajectories {

namespace mtg = mav_trajectory_generation;

CircleTrajectory::Settings::Settings(
    double center_east, double center_north, double radius,
    double start_heading_deg, double arc_length_deg_in, double altitude,
    double velocity_in, double offset_heading_deg,
    const std::weak_ptr<BaseTrajectory>& prev_trajectory,
    const mtg::InputConstraints& input_constraints,
    double circle_deviation_ratio /* = 0.01 */)
    : BaseTrajectory::Settings(input_constraints, velocity_in,
                               offset_heading_deg, prev_trajectory),
      center_east(center_east),
      center_north(center_north),
      radius(std::max(std::fabs(radius), kZeroGuard)),
      start_heading_deg(start_heading_deg),
      arc_length_deg(std::max(std::fabs(arc_length_deg_in), kMinArcLength)),
      altitude(altitude),
      circle_deviation_ratio(
          std::max(std::min(std::fabs(circle_deviation_ratio), 0.999), 0.001)) {
  // Arc feasibility.
  ROS_WARN_COND(arc_length_deg_in < 0.0,
                "Input arc length negative. Inverted.");
  if (std::fabs(arc_length_deg_in) < kMinArcLength)
    ROS_WARN_STREAM("Arc length too small: " << arc_length_deg_in
                                             << " degrees. Clipped to: "
                                             << arc_length_deg << " degrees.");

  // Yaw rate feasibility.
  double omega_z_max = std::numeric_limits<double>::max();
  if (input_constraints.getConstraint(
          mav_trajectory_generation::InputConstraintType::kOmegaZMax,
          &omega_z_max)) {
    if (computeAngularVelocity() > omega_z_max) {
      double new_velocity =
          std::copysign(omega_z_max * radius - kVelocityResolution, velocity);
      ROS_WARN_STREAM("Trajectory yaw rate too large: "
                      << computeAngularVelocity()
                      << " rad/s. Clipping velocity to: " << new_velocity
                      << " m/s");
      velocity = new_velocity;
    }
  }
}

double CircleTrajectory::Settings::computeAngularVelocity() const {
  return velocity / radius;
}

CircleTrajectory::CircleTrajectory(
    const std::shared_ptr<CircleTrajectory::Settings>& settings)
    : BaseTrajectory(settings),
      start_rad_(angles::from_degrees(
          std::static_pointer_cast<CircleTrajectory::Settings>(settings_)
              ->start_heading_deg -
          90)),
      arc_length_rad_(angles::from_degrees(
          std::static_pointer_cast<CircleTrajectory::Settings>(settings_)
              ->arc_length_deg)) {
  // Angle between vertices.
  // https://stackoverflow.com/questions/11774038/how-to-render-a-circle-with-as-few-vertices-as-possible
  const double kDeltaAngleMax = std::acos(
      2.0 * std::pow(
                (1.0 -
                 std::static_pointer_cast<CircleTrajectory::Settings>(settings_)
                     ->circle_deviation_ratio),
                2.0) -
      1.0);
  const double kNumVerticesMin = std::fabs(arc_length_rad_) / kDeltaAngleMax;
  num_vertices_ = std::ceil(kNumVerticesMin);
  num_vertices_ += 1;  // To close circle.
}

bool CircleTrajectory::toYaml(YAML::Node* node) {
  CHECK_NOTNULL(node);
  *node = YAML::Node();

  Eigen::Vector3d center_wgs84;
  if (!geotf_.convert(
          "enu",
          Eigen::Vector3d(
              std::static_pointer_cast<CircleTrajectory::Settings>(settings_)
                  ->center_east,
              std::static_pointer_cast<CircleTrajectory::Settings>(settings_)
                  ->center_north,
              std::static_pointer_cast<CircleTrajectory::Settings>(settings_)
                  ->altitude),
          "wgs84", &center_wgs84))
    return false;

  (*node)["altitude"] =
      std::static_pointer_cast<CircleTrajectory::Settings>(settings_)->altitude;
  (*node)["velocity"] =
      std::static_pointer_cast<CircleTrajectory::Settings>(settings_)->velocity;
  (*node)["trajectory_type"] = static_cast<int>(TrajectoryType::Hotpoint);
  (*node)["center_lat"] = center_wgs84.x();
  (*node)["center_lon"] = center_wgs84.y();
  (*node)["radius"] =
      std::static_pointer_cast<CircleTrajectory::Settings>(settings_)->radius;
  (*node)["start_heading_deg"] =
      std::static_pointer_cast<CircleTrajectory::Settings>(settings_)
          ->start_heading_deg;
  (*node)["arc_length_deg"] =
      std::static_pointer_cast<CircleTrajectory::Settings>(settings_)
          ->arc_length_deg;
  (*node)["offset_heading_deg"] =
      std::static_pointer_cast<CircleTrajectory::Settings>(settings_)
          ->offset_heading_deg;
  (*node)["circle_deviation_ratio"] =
      std::static_pointer_cast<CircleTrajectory::Settings>(settings_)
          ->circle_deviation_ratio;

  return true;
}

void CircleTrajectory::samplePositionVertices() {
  const Eigen::Vector3d kOffset(
      std::static_pointer_cast<CircleTrajectory::Settings>(settings_)
          ->center_east,
      std::static_pointer_cast<CircleTrajectory::Settings>(settings_)
          ->center_north,
      std::static_pointer_cast<CircleTrajectory::Settings>(settings_)
          ->altitude);
  const double kAngularVelocity =
      std::static_pointer_cast<CircleTrajectory::Settings>(settings_)
          ->computeAngularVelocity();
  const double kDeltaAngle = computeDeltaAngle(kAngularVelocity);

  position_vertices_.resize(num_vertices_, kPositionDimension);
  for (int i = 0; i < num_vertices_; ++i) {
    const double kAngle = i * kDeltaAngle + start_rad_;

    Eigen::Vector3d position;
    position.x() =
        std::static_pointer_cast<CircleTrajectory::Settings>(settings_)
            ->radius *
        std::cos(kAngle);
    position.y() =
        std::static_pointer_cast<CircleTrajectory::Settings>(settings_)
            ->radius *
        std::sin(kAngle);
    position.z() = 0.0;
    if (kMaxDerivativePos >= mtg::derivative_order::POSITION) {
      position_vertices_[i].addConstraint(mtg::derivative_order::POSITION,
                                          position + kOffset);
    }

    Eigen::Vector3d velocity;
    velocity.x() =
        -std::static_pointer_cast<CircleTrajectory::Settings>(settings_)
             ->radius *
        kAngularVelocity * std::sin(kAngle);
    velocity.y() =
        std::static_pointer_cast<CircleTrajectory::Settings>(settings_)
            ->radius *
        kAngularVelocity * std::cos(kAngle);
    velocity.z() = 0.0;
    if (kMaxDerivativePos >= mtg::derivative_order::VELOCITY) {
      position_vertices_[i].addConstraint(mtg::derivative_order::VELOCITY,
                                          velocity);
    }

    if (kMaxDerivativePos >= mtg::derivative_order::ACCELERATION) {
      Eigen::Vector3d acceleration;
      acceleration.x() = -std::pow(kAngularVelocity, 2.0) * position.x();
      acceleration.y() = -std::pow(kAngularVelocity, 2.0) * position.y();
      acceleration.z() = 0.0;
      position_vertices_[i].addConstraint(mtg::derivative_order::ACCELERATION,
                                          acceleration);
    }

    if (kMaxDerivativePos >= mtg::derivative_order::JERK) {
      Eigen::Vector3d jerk;
      jerk.x() = -std::pow(kAngularVelocity, 3.0) * velocity.x();
      jerk.y() = -std::pow(kAngularVelocity, 3.0) * velocity.y();
      jerk.z() = 0.0;
      position_vertices_[i].addConstraint(mtg::derivative_order::JERK, jerk);
    }

    if (kMaxDerivativePos >= mtg::derivative_order::SNAP) {
      Eigen::Vector3d snap;
      snap.x() = std::pow(kAngularVelocity, 4.0) * position.x();
      snap.y() = std::pow(kAngularVelocity, 4.0) * position.y();
      snap.z() = 0.0;
      position_vertices_[i].addConstraint(mtg::derivative_order::SNAP, snap);
    }
  }
}

void CircleTrajectory::sampleYawVertices() {
  const double kOffsetHeading = angles::from_degrees(
      std::static_pointer_cast<CircleTrajectory::Settings>(settings_)
          ->offset_heading_deg);
  const double kAngularVelocity =
      std::static_pointer_cast<CircleTrajectory::Settings>(settings_)
          ->computeAngularVelocity();
  const double kDeltaAngle = computeDeltaAngle(kAngularVelocity);

  yaw_vertices_.resize(num_vertices_, kYawDimension);
  for (int i = 0; i < num_vertices_; ++i) {
    typedef Eigen::Matrix<double, 1, 1> Vector1d;
    if (kMaxDerivativeYaw >= mtg::derivative_order::ORIENTATION) {
      Vector1d orientation;
      const double kAngle = i * kDeltaAngle + start_rad_;
      orientation[0] = kAngle + 0.5 * M_PI + kOffsetHeading;
      yaw_vertices_[i].addConstraint(mtg::derivative_order::ORIENTATION,
                                     orientation);
    }

    if (kMaxDerivativeYaw >= mtg::derivative_order::ANGULAR_VELOCITY) {
      Vector1d angular_velocity;
      angular_velocity[0] = kAngularVelocity;
      yaw_vertices_[i].addConstraint(mtg::derivative_order::ANGULAR_VELOCITY,
                                     angular_velocity);
    }

    if (kMaxDerivativeYaw >= mtg::derivative_order::ANGULAR_ACCELERATION) {
      Vector1d angular_acceleration;
      angular_acceleration[0] = 0.0;
      yaw_vertices_[i].addConstraint(
          mtg::derivative_order::ANGULAR_ACCELERATION, angular_acceleration);
    }
  }
}

double CircleTrajectory::computeDeltaAngle(double angular_velocity) const {
  return std::copysign(arc_length_rad_ / (num_vertices_ - 1), angular_velocity);
}

void CircleTrajectory::sampleTimes() {
  const double kAngularVelocity =
      std::static_pointer_cast<CircleTrajectory::Settings>(settings_)
          ->computeAngularVelocity();
  const double kDeltaAngle = computeDeltaAngle(kAngularVelocity);
  const double kSamplePeriod = std::fabs(kDeltaAngle / kAngularVelocity);

  segment_times_.resize(position_vertices_.size() - 1);
  for (double& t : segment_times_) t = kSamplePeriod;
}

}  // namespace fm_trajectories
