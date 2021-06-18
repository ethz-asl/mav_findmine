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

#include "fm_trajectories/polygon_trajectory.h"

#include <angles/angles.h>

namespace fm_trajectories {

PolygonTrajectory::Settings::Settings(
    const std::vector<Eigen::Vector3d>& vertices_in, double velocity,
    double offset_heading_deg,
    const std::weak_ptr<BaseTrajectory>& prev_trajectory,
    const mtg::InputConstraints& input_constraints, size_t start_vertex,
    size_t num_edges)
    : BaseTrajectory::Settings(input_constraints, velocity, offset_heading_deg,
                               prev_trajectory),
      vertices(vertices_in),
      start_vertex(start_vertex % vertices.size()),
      num_edges(std::max(static_cast<size_t>(1), num_edges)),
      is_waypoint_list(false) {
  // Rotate vertices according to start vertex.
  std::rotate(vertices.begin(), vertices.begin() + start_vertex,
              vertices.end());
  // Change direction for negative velocity.
  if (velocity < 0.0) {
    std::reverse(vertices.begin(), vertices.end());
    std::rotate(vertices.begin(), std::prev(vertices.end()), vertices.end());
  }
  edges = computeEdges(vertices);
}

PolygonTrajectory::Settings::Edge::Edge(const Eigen::Vector3d& from,
                                        const Eigen::Vector3d& to) {
  direction = to - from;
  length = direction.norm();
  direction /= length;
  angle = std::atan2(direction.y(), direction.x());
}

std::vector<PolygonTrajectory::Settings::Edge>
PolygonTrajectory::Settings::computeEdges(
    const std::vector<Eigen::Vector3d>& vertices) const {
  std::vector<Edge> edges(vertices.size());

  for (size_t i = 0; i < vertices.size(); ++i) {
    edges[i] = Edge(vertices[i], vertices[(i + 1) % vertices.size()]);
  }
  return edges;
}

PolygonTrajectory::PolygonTrajectory(
    const std::shared_ptr<PolygonTrajectory::Settings>& settings)
    : BaseTrajectory(settings) {}

bool PolygonTrajectory::toYaml(YAML::Node* node) {
  CHECK_NOTNULL(node);
  *node = YAML::Node();

  for (const Eigen::Vector3d& v :
       std::static_pointer_cast<PolygonTrajectory::Settings>(settings_)
           ->vertices) {
    Eigen::Vector3d v_wgs84;
    if (!geotf_.convert("enu", v, "wgs84", &v_wgs84)) return false;
    double lat, lon, alt;
    (*node)["latitude"].push_back(v_wgs84.x());
    (*node)["longitude"].push_back(v_wgs84.y());
    (*node)["altitude"].push_back(v.z());
  }

  (*node)["velocity"] =
      std::static_pointer_cast<PolygonTrajectory::Settings>(settings_)
          ->velocity;
  (*node)["trajectory_type"] = static_cast<int>(TrajectoryType::Polygon);
  (*node)["num_edges"] =
      std::static_pointer_cast<PolygonTrajectory::Settings>(settings_)
          ->num_edges;
  (*node)["start_vertex"] =
      std::static_pointer_cast<PolygonTrajectory::Settings>(settings_)
          ->start_vertex;

  return true;
}

// Constant velocity -> decelerate -> turn -> accelerate -> constant velocity ->
// ... -> constant velocity
void PolygonTrajectory::samplePositionVertices() {
  position_vertices_.clear();
  const std::vector<Eigen::Vector3d>& vertices =
      std::static_pointer_cast<PolygonTrajectory::Settings>(settings_)
          ->vertices;
  const std::vector<Settings::Edge>& edges =
      std::static_pointer_cast<PolygonTrajectory::Settings>(settings_)->edges;
  double velocity =
      std::static_pointer_cast<PolygonTrajectory::Settings>(settings_)
          ->velocity;
  size_t num_edges =
      std::static_pointer_cast<PolygonTrajectory::Settings>(settings_)
          ->num_edges;
  bool is_waypoint_list =
      std::static_pointer_cast<PolygonTrajectory::Settings>(settings_)
          ->is_waypoint_list;

  size_t N = edges.size();
  Eigen::Vector3d vertex = vertices.front();
  for (size_t i = 0; i < num_edges; ++i) {
    // Waypoint list. Reset to start waypoint.
    if (is_waypoint_list && (i % N) == 0) vertex = vertices.front();
    // Add constant velocity segments.
    mtg::Vertex v(kPositionDimension);
    v.makeStartOrEnd(vertex, kMaxDerivativePos);
    Eigen::Vector3d vel = std::fabs(velocity) * edges[i % N].direction;
    v.addConstraint(mtg::derivative_order::VELOCITY, vel);
    position_vertices_.push_back(v);

    vertex += edges[i % N].direction * edges[i % N].length;
    v.addConstraint(mtg::derivative_order::POSITION, vertex);
    position_vertices_.push_back(v);

    if (i == num_edges - 1) continue;  // Not for last vertex.
    // Add deceleration segment.
    v.makeStartOrEnd(vertex, kMaxDerivativePos);
    position_vertices_.push_back(v);

    // Turning or returning vertex.
    if (is_waypoint_list && (i % N) == 0) {
      // Return to start.
      v.makeStartOrEnd(vertices.front(), kMaxDerivativePos);
      position_vertices_.push_back(v);
    } else {
      // Turn on spot.
      position_vertices_.push_back(v);
    }
  }
}

void PolygonTrajectory::sampleYawVertices() {
  yaw_vertices_.clear();
  typedef Eigen::Matrix<double, 1, 1> Vector1d;
  double offset_heading = angles::from_degrees(
      std::static_pointer_cast<PolygonTrajectory::Settings>(settings_)
          ->offset_heading_deg);
  const std::vector<Settings::Edge>& edges =
      std::static_pointer_cast<PolygonTrajectory::Settings>(settings_)->edges;
  double velocity =
      std::static_pointer_cast<PolygonTrajectory::Settings>(settings_)
          ->velocity;
  size_t num_edges =
      std::static_pointer_cast<PolygonTrajectory::Settings>(settings_)
          ->num_edges;

  size_t N = edges.size();
  for (size_t i = 0; i < num_edges; ++i) {
    // Add angle for constant velocity segments.
    double yaw = edges[i % N].angle + offset_heading;
    if (velocity < 0.0) yaw += M_PI;
    mtg::Vertex v(kYawDimension);
    v.makeStartOrEnd(yaw, kMaxDerivativeYaw);
    yaw_vertices_.push_back(v);
    yaw_vertices_.push_back(v);

    if (i == num_edges - 1) continue;  // Not for last vertex.
    // Add angle for deceleration segment.
    yaw_vertices_.push_back(v);

    yaw = edges[(i + 1) % N].angle + offset_heading;
    if (velocity < 0.0) yaw += M_PI;
    v.makeStartOrEnd(yaw, kMaxDerivativeYaw);
    yaw_vertices_.push_back(v);
  }
}

// The segment time is only defined for the edges.
void PolygonTrajectory::sampleTimes() {
  CHECK_EQ(position_vertices_.size(), yaw_vertices_.size());
  CHECK_GT(position_vertices_.size(), 1);
  segment_times_.clear();

  size_t num_edges =
      std::static_pointer_cast<PolygonTrajectory::Settings>(settings_)
          ->num_edges;
  const std::vector<Settings::Edge>& edges =
      std::static_pointer_cast<PolygonTrajectory::Settings>(settings_)->edges;
  double velocity =
      std::fabs(std::static_pointer_cast<PolygonTrajectory::Settings>(settings_)
                    ->velocity);
  const mtg::FeasibilityRecursive& feasibility_check =
      std::static_pointer_cast<PolygonTrajectory::Settings>(settings_)
          ->feasibility_check;
  bool is_waypoint_list =
      std::static_pointer_cast<PolygonTrajectory::Settings>(settings_)
          ->is_waypoint_list;

  // Find acceleration time.
  double acceleration_time = 0.0;
  if (num_edges > 1) {
    CHECK_GT(position_vertices_.size(), 2);
    CHECK_GT(yaw_vertices_.size(), 2);
    mtg::Vertex::Vector pos_vertices = {position_vertices_[1],
                                        position_vertices_[2]};
    mtg::Vertex::Vector yaw_vertices = {yaw_vertices_[1], yaw_vertices_[2]};
    if (!searchSegmentTimeLinear(
            pos_vertices, yaw_vertices, kPosDerivativeToOptimize,
            kYawDerivativeToOptimize, feasibility_check, &acceleration_time)) {
      ROS_ERROR("Failed to get acceleration segment times.");
      return;
    }
  }

  // Find turning time.
  double turning_time = 0.0;
  if (edges.size() > 1 && num_edges != 1) {
    CHECK_GT(position_vertices_.size(), 3);
    CHECK_GT(yaw_vertices_.size(), 3);
    mtg::Vertex::Vector pos_vertices = {position_vertices_[2],
                                        position_vertices_[3]};
    mtg::Vertex::Vector yaw_vertices = {yaw_vertices_[2], yaw_vertices_[3]};
    if (!searchSegmentTimeLinear(
            pos_vertices, yaw_vertices, kPosDerivativeToOptimize,
            kYawDerivativeToOptimize, feasibility_check, &turning_time)) {
      ROS_ERROR("Failed to get turning segment times.");
      return;
    }
  }

  // Find returning time.
  double returning_time = 0.0;
  if (is_waypoint_list) {
    CHECK_GT(position_vertices_.size(), 1);
    CHECK_GT(yaw_vertices_.size(), 1);
    mtg::Vertex::Vector pos_vertices = {position_vertices_.back(),
                                        position_vertices_.front()};
    mtg::Vertex::Vector yaw_vertices = {yaw_vertices_.back(),
                                        yaw_vertices_.front()};
    if (!searchSegmentTimeLinear(
            pos_vertices, yaw_vertices, kPosDerivativeToOptimize,
            kYawDerivativeToOptimize, feasibility_check, &returning_time)) {
      ROS_ERROR("Failed to get returning segment times.");
      return;
    }
  }

  size_t N = edges.size();
  for (size_t i = 0; i < num_edges; ++i) {
    // Add time for constant velocity segments.
    double constant_time = edges[i % N].length / velocity;
    segment_times_.push_back(constant_time);

    if (i == num_edges - 1) continue;             // Not for last edge.
    segment_times_.push_back(acceleration_time);  // Stop.
    if (is_waypoint_list && (i % N) == 0) {
      segment_times_.push_back(returning_time);
    } else {
      segment_times_.push_back(turning_time);
    }
    segment_times_.push_back(acceleration_time);  // Accelerate next edge.
  }
}

}  // namespace fm_trajectories
