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

#include "fm_trajectories/waypoint_trajectory.h"

#include <angles/angles.h>

namespace fm_trajectories {

WaypointTrajectory::Settings::Settings(
    const std::vector<Eigen::Vector3d>& vertices_in, double velocity,
    double offset_heading_deg,
    const std::weak_ptr<BaseTrajectory>& prev_trajectory,
    const mtg::InputConstraints& input_constraints)
    : BaseTrajectory::Settings(input_constraints, velocity, offset_heading_deg,
                               prev_trajectory),
      vertices(vertices_in) {
  // Cleanup redundant vertices.
  bool is_clean = false;
  while (!is_clean) {
    for (std::vector<Eigen::Vector3d>::iterator it = vertices.begin();
         it != std::prev(vertices.end()); ++it) {
      if (*it == *std::next(it)) {
        ROS_WARN_STREAM(
            "Removing redundant vertex: " << std::next(it)->transpose());
        vertices.erase(std::next(it));
        break;
      }
      if (it == std::prev(vertices.end(), 2)) is_clean = true;
    }
  }

  // Change direction for negative velocity.
  if (velocity < 0.0) {
    std::reverse(vertices.begin(), vertices.end());
    std::rotate(vertices.begin(), std::prev(vertices.end()), vertices.end());
  }
  velocity = std::fabs(velocity);
  edges = computeEdges(vertices);

  CHECK_EQ(edges.size(), vertices.size() - 1);
}

WaypointTrajectory::Settings::Edge::Edge(const Eigen::Vector3d& from,
                                         const Eigen::Vector3d& to) {
  direction = to - from;
  length = direction.norm();
  direction /= length;
  angle = std::atan2(direction.y(), direction.x());
}

std::vector<WaypointTrajectory::Settings::Edge>
WaypointTrajectory::Settings::computeEdges(
    const std::vector<Eigen::Vector3d>& vertices) const {
  std::vector<Edge> edges(vertices.size() - 1);

  for (size_t i = 0; i < vertices.size() - 1; ++i) {
    edges[i] = Edge(vertices[i], vertices[i + 1]);
  }
  return edges;
}

WaypointTrajectory::WaypointTrajectory(
    const std::shared_ptr<WaypointTrajectory::Settings>& settings)
    : BaseTrajectory(settings) {}

bool WaypointTrajectory::toYaml(YAML::Node* node) {
  CHECK_NOTNULL(node);
  *node = YAML::Node();

  for (const Eigen::Vector3d& v :
       std::static_pointer_cast<WaypointTrajectory::Settings>(settings_)
           ->vertices) {
    Eigen::Vector3d v_wgs84;
    if (!geotf_.convert("enu", v, "wgs84", &v_wgs84)) return false;

    (*node)["latitude"].push_back(v_wgs84.x());
    (*node)["longitude"].push_back(v_wgs84.y());
    (*node)["altitude"].push_back(v.z());
  }

  (*node)["velocity"] =
      std::static_pointer_cast<WaypointTrajectory::Settings>(settings_)
          ->velocity;
  (*node)["trajectory_type"] = static_cast<int>(TrajectoryType::Waypoints);

  return true;
}

// Rest-to-rest.
void WaypointTrajectory::samplePositionVertices() {
  position_vertices_.clear();
  const std::vector<Eigen::Vector3d>& vertices =
      std::static_pointer_cast<WaypointTrajectory::Settings>(settings_)
          ->vertices;
  const std::vector<Settings::Edge>& edges =
      std::static_pointer_cast<WaypointTrajectory::Settings>(settings_)->edges;
  double v_max =
      std::static_pointer_cast<WaypointTrajectory::Settings>(settings_)
          ->velocity;
  // TODO(rikba): A little bit hacky way of defining a_max.
  const mtg::FeasibilityRecursive& feasibility_check =
      std::static_pointer_cast<WaypointTrajectory::Settings>(settings_)
          ->feasibility_check;
  double a_max;
  feasibility_check.getInputConstraints().getConstraint(
      mav_trajectory_generation::InputConstraintType::kFMax, &a_max);
  a_max -= 9.81;
  a_max *= 0.5;

  for (size_t i = 0; i < vertices.size() - 1; ++i) {
    // Turn at rest.
    mtg::Vertex v_from(kPositionDimension);
    v_from.makeStartOrEnd(vertices[i], kMaxDerivativePos);
    map_pos_v_to_edge_[position_vertices_.size()] = i;
    position_vertices_.push_back(v_from);
    vertex_types_.push_back(VertexType::kTurn);
    // Acceleration.
    mtg::Vertex v_acc(kPositionDimension);
    double acc_distance = 0.0;
    bool has_max_vel = computeAccelerationDistance(vertices[i], vertices[i + 1],
                                                   v_max, a_max, &acc_distance);
    Eigen::Vector3d acc_position =
        vertices[i] + acc_distance * edges[i].direction;
    v_acc.makeStartOrEnd(acc_position, kMaxDerivativePos);
    if (has_max_vel)
      v_acc.addConstraint(mtg::derivative_order::VELOCITY,
                          edges[i].direction * v_max);
    else
      v_acc.addConstraint(
          mtg::derivative_order::VELOCITY,
          edges[i].direction * std::sqrt(2.0 * a_max * acc_distance));
    map_pos_v_to_edge_[position_vertices_.size()] = i;
    position_vertices_.push_back(v_acc);
    vertex_types_.push_back(VertexType::kVelStart);
    // Constant velocity.
    if (has_max_vel) {
      mtg::Vertex v_finish_vel(kPositionDimension);
      Eigen::Vector3d dec_position =
          vertices[i + 1] - acc_distance * edges[i].direction;
      v_finish_vel.makeStartOrEnd(dec_position, kMaxDerivativePos);
      v_finish_vel.addConstraint(mtg::derivative_order::VELOCITY,
                                 edges[i].direction * v_max);
      map_pos_v_to_edge_[position_vertices_.size()] = i;
      position_vertices_.push_back(v_finish_vel);
      vertex_types_.push_back(VertexType::kVelEnd);
    }
    // Rest.
    mtg::Vertex v_to(kPositionDimension);
    v_to.makeStartOrEnd(vertices[i + 1], kMaxDerivativePos);
    map_pos_v_to_edge_[position_vertices_.size()] = i;
    position_vertices_.push_back(v_to);
    vertex_types_.push_back(VertexType::kStop);
  }
}

void WaypointTrajectory::sampleYawVertices() {
  yaw_vertices_.clear();
  typedef Eigen::Matrix<double, 1, 1> Vector1d;
  double offset_heading = angles::from_degrees(
      std::static_pointer_cast<WaypointTrajectory::Settings>(settings_)
          ->offset_heading_deg);
  const std::vector<Settings::Edge>& edges =
      std::static_pointer_cast<WaypointTrajectory::Settings>(settings_)->edges;
  double velocity = std::fabs(
      std::static_pointer_cast<WaypointTrajectory::Settings>(settings_)
          ->velocity);

  CHECK_EQ(position_vertices_.size(), vertex_types_.size());
  for (size_t i = 0; i < position_vertices_.size(); ++i) {
    if (vertex_types_[i] == VertexType::kTurn) {
      // Turn to flight direction at rest.
      double yaw = edges[map_pos_v_to_edge_[i]].angle + offset_heading;
      if (velocity < 0.0) yaw += M_PI;
      mtg::Vertex v(kYawDimension);
      v.makeStartOrEnd(yaw, kMaxDerivativeYaw);
      yaw_vertices_.push_back(v);
    } else {
      // Keep constant yaw.
      CHECK_LT(i - 1, yaw_vertices_.size());
      yaw_vertices_.push_back(yaw_vertices_[i - 1]);
    }
  }
}

// The segment time is only defined for the edges.
void WaypointTrajectory::sampleTimes() {
  CHECK_EQ(position_vertices_.size(), yaw_vertices_.size());
  CHECK_GT(position_vertices_.size(), 1);
  segment_times_.clear();

  const std::vector<Settings::Edge>& edges =
      std::static_pointer_cast<WaypointTrajectory::Settings>(settings_)->edges;
  double v_max = std::fabs(
      std::static_pointer_cast<WaypointTrajectory::Settings>(settings_)
          ->velocity);
  const mtg::FeasibilityRecursive& feasibility_check =
      std::static_pointer_cast<WaypointTrajectory::Settings>(settings_)
          ->feasibility_check;

  // TODO(rikba): A bit hacky way of defining a_max.
  double a_max;
  feasibility_check.getInputConstraints().getConstraint(
      mav_trajectory_generation::InputConstraintType::kFMax, &a_max);
  a_max -= 9.81;
  a_max *= 0.5;

  segment_times_.resize(position_vertices_.size() - 1);
  for (size_t i = 0; i < segment_times_.size(); i++) {
    if ((vertex_types_[i] == VertexType::kTurn) &&
        (vertex_types_[i + 1] == VertexType::kVelStart)) {
      // Acceleration segment.
      mtg::Vertex::Vector pos_vertices = {position_vertices_[i],
                                          position_vertices_[i + 1]};
      mtg::Vertex::Vector yaw_vertices = {yaw_vertices_[i],
                                          yaw_vertices_[i + 1]};
      if (!searchSegmentTimeLinear(pos_vertices, yaw_vertices,
                                   kPosDerivativeToOptimize,
                                   kYawDerivativeToOptimize, feasibility_check,
                                   &segment_times_[i])) {
        ROS_ERROR("Failed to get acceleration segment time.");
        return;
      }
    } else if ((vertex_types_[i] == VertexType::kVelStart) &&
               (vertex_types_[i + 1] == VertexType::kVelEnd)) {
      // Constant velocity segment.
      Eigen::VectorXd vel_start, vel_finish;
      position_vertices_[i].getConstraint(mtg::derivative_order::POSITION,
                                          &vel_start);
      position_vertices_[i + 1].getConstraint(mtg::derivative_order::POSITION,
                                              &vel_finish);
      segment_times_[i] = (vel_finish - vel_start).norm() / v_max;
    } else if ((vertex_types_[i] == VertexType::kVelEnd) &&
               (vertex_types_[i + 1] == VertexType::kStop)) {
      // Deceleration segment 1. Time is same as acceleration.
      CHECK_LT(i - 2, segment_times_.size());
      segment_times_[i] = segment_times_[i - 2];
    } else if ((vertex_types_[i] == VertexType::kVelStart) &&
               (vertex_types_[i + 1] == VertexType::kStop)) {
      // Deceleration segment 2. Time is same as acceleration.
      CHECK_LT(i - 1, segment_times_.size());
      segment_times_[i] = segment_times_[i - 1];
    } else if ((vertex_types_[i] == VertexType::kStop) &&
               (vertex_types_[i + 1] == VertexType::kTurn)) {
      CHECK_LT(i + 1, position_vertices_.size());
      CHECK_LT(i + 1, yaw_vertices_.size());
      mtg::Vertex::Vector pos_vertices = mtg::Vertex::Vector(
          {position_vertices_[i], position_vertices_[i + 1]});
      mtg::Vertex::Vector yaw_vertices =
          mtg::Vertex::Vector({yaw_vertices_[i], yaw_vertices_[i + 1]});
      if (!searchSegmentTimeLinear(pos_vertices, yaw_vertices,
                                   kPosDerivativeToOptimize,
                                   kYawDerivativeToOptimize, feasibility_check,
                                   &segment_times_[i])) {
        ROS_ERROR("Failed to get turning segment time.");
        return;
      }
    } else {
      CHECK(false) << "Invalid segment time case. This type: "
                   << vertex_types_[i]
                   << " next type: " << vertex_types_[i + 1];
    }
  }
}

bool WaypointTrajectory::computeAccelerationDistance(
    const Eigen::Vector3d& from, const Eigen::Vector3d& to, double v_max,
    double a_max, double* acc_distance) {
  const double distance = (to - from).norm();
  // Time to accelerate or decelerate to or from maximum velocity:
  const double acc_time = v_max / a_max;
  // Distance covered during complete acceleration or decelerate:
  *acc_distance = 0.5 * v_max * acc_time;
  if (distance < 2.0 * (*acc_distance)) {
    // Case 1: Distance too small to accelerate to maximum velocity.
    *acc_distance = 0.5 * distance;
    return false;
  } else {
    // Case 2: Accelerate to maximum velocity.
    return true;
  }
}

}  // namespace fm_trajectories
