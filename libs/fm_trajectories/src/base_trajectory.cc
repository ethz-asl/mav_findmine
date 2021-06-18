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

#include "fm_trajectories/base_trajectory.h"

#include <angles/angles.h>
#include <mav_trajectory_generation/trajectory.h>
#include <mav_trajectory_generation/trajectory_sampling.h>
#include <mav_trajectory_generation/vertex.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <ros/ros.h>

#include <mav_trajectory_generation/io.h>

namespace fm_trajectories {

BaseTrajectory::Settings::Settings(
    const mtg::InputConstraints& input_constraints, double velocity_in,
    double offset_heading_deg,
    const std::weak_ptr<BaseTrajectory> prev_trajectory,
    bool constant_velocity /* = true */)
    : feasibility_check(input_constraints),
      velocity(velocity_in),
      offset_heading_deg(offset_heading_deg),
      prev_trajectory(prev_trajectory),
      constant_velocity(constant_velocity) {
  // Velocity feasibility.
  if (std::fabs(velocity) < kMinVelocity) {
    double new_velocity = std::copysign(kMinVelocity, velocity);
    ROS_WARN_STREAM("Trajectory velocity too small: "
                    << velocity << ". Clipping to: " << new_velocity << " m/s");
    velocity = new_velocity;
  }

  double v_max = std::numeric_limits<double>::max();
  if (input_constraints.getConstraint(
          mav_trajectory_generation::InputConstraintType::kVMax, &v_max)) {
    if (std::fabs(velocity) >= v_max) {
      double new_velocity =
          std::copysign(v_max - kVelocityResolution, velocity);
      ROS_WARN_STREAM("Trajectory velocity too large: "
                      << velocity << " m/s. Clipping to: " << new_velocity
                      << " m/s");
      velocity = new_velocity;
    }
  }
}

BaseTrajectory::BaseTrajectory(const mtg::Trajectory& trajectory,
                               const std::shared_ptr<Settings>& settings)
    : BaseTrajectory(settings) {
  // Get vertices and segment times from trajectory.
  CHECK(trajectory.getVertices(kMaxDerivativePos, kMaxDerivativeYaw,
                               &position_vertices_, &yaw_vertices_));
  segment_times_ = trajectory.getSegmentTimes();
}

void BaseTrajectory::setGeodeticReference(double lat, double lon, double alt) {
  geotf_.removeFrame("enu");
  geotf_.addFrameByENUOrigin("enu", lat, lon, alt);
}

visualization_msgs::MarkerArray BaseTrajectory::getTrajectoryDeleteMarkers() {
  visualization_msgs::MarkerArray markers = getTrajectoryMarkers();
  for (visualization_msgs::Marker& m : markers.markers)
    m.action = visualization_msgs::Marker::DELETE;
  return markers;
}

bool BaseTrajectory::planTrajectory(mtg::Trajectory* trajectory,
                                    bool use_cache /* = true */) {
  CHECK_NOTNULL(trajectory);
  trajectory->clear();

  if (!trajectory_cache_.empty() && use_cache) {
    *trajectory = trajectory_cache_;
    return true;
  }

  if (!computeTrajectory(trajectory)) {
    ROS_ERROR("Could not compute initial trajectory.");
    return false;
  }

  // // Binary search for feasible velocity.
  // mtg::InputFeasibilityResult result =
  //     settings_->feasibility_check.checkInputFeasibilityTrajectory(*trajectory);
  // if (result != mtg::InputFeasibilityResult::kInputFeasible) {
  //   ROS_WARN_STREAM("The desired trajectory is infeasible: "
  //                   << mtg::getInputFeasibilityResultName(result));
  //   ROS_WARN("Reducing velocity.");
  //   double v_max = settings_->velocity;
  //   double v_min = std::copysign(kMinVelocity, settings_->velocity);
  //   bool is_feasible = false;
  //   while (std::fabs(v_max - v_min) >= kVelocityResolution) {
  //     settings_->velocity = (v_max + v_min) * 0.5;
  //     mtg::Trajectory temp_trajectory;
  //     if (!computeTrajectory(&temp_trajectory)) {
  //       ROS_ERROR("Could not compute binary search trajectory.");
  //       return false;
  //     }
  //     result = settings_->feasibility_check.checkInputFeasibilityTrajectory(
  //         temp_trajectory);
  //     if (result == mtg::InputFeasibilityResult::kInputFeasible) {
  //       v_min = settings_->velocity;
  //       *trajectory = temp_trajectory;
  //       is_feasible = true;
  //     } else {
  //       v_max = settings_->velocity;
  //     }
  //   }
  //   settings_->velocity = v_min;
  //   if (!is_feasible) {
  //     ROS_ERROR("Binary search for feasible velocity failed.");
  //     return false;
  //   }
  //   ROS_WARN_STREAM("New velocity: " << settings_->velocity);
  // }

  trajectory_cache_ = *trajectory;

  return true;
}

bool BaseTrajectory::sampleVertices() {
  samplePositionVertices();
  sampleYawVertices();
  sampleTimes();

  if (settings_->constant_velocity) {
    addAccelerationVertices();
  }
  return connectToPrevious();
}

bool BaseTrajectory::computeTrajectory(mtg::Trajectory* trajectory) {
  CHECK_NOTNULL(trajectory);
  trajectory->clear();

  if (!sampleVertices()) return false;

  // Get polynomial trajectory.
  mtg::Trajectory pos_trajectory;
  if (!linearOptimization<kNPos>(position_vertices_, segment_times_,
                                 kPosDerivativeToOptimize, &pos_trajectory)) {
    ROS_ERROR("Failed linear position optimization.");
    return false;
  }
  mtg::Trajectory yaw_trajectory;
  if (!linearYawOptimization(yaw_vertices_, segment_times_,
                             kYawDerivativeToOptimize, &yaw_trajectory)) {
    ROS_ERROR("Failed linear yaw optimization.");
    return false;
  }
  if (!pos_trajectory.getTrajectoryWithAppendedDimension(yaw_trajectory,
                                                         trajectory)) {
    ROS_ERROR("Failed to merge position and yaw trajectory.");
    return false;
  }

  return true;
}

bool BaseTrajectory::linearYawOptimization(
    const mtg::Vertex::Vector& yaw_vertices,
    const std::vector<double>& segment_times, int derivative_to_optimize,
    mtg::Trajectory* trajectory) {
  ROS_ASSERT(trajectory);
  trajectory->clear();

  // Find closest equivalent yaw angle.
  // Note: Does not consider yaw rate.
  mtg::Vertex::Vector corrected_yaw_vertices = yaw_vertices;
  for (size_t i = 0; i < corrected_yaw_vertices.size() - 1; ++i) {
    // Get from yaw angle.
    Eigen::VectorXd from_yaw;
    if (!corrected_yaw_vertices[i].getConstraint(
            mtg::derivative_order::ORIENTATION, &from_yaw)) {
      continue;
    }
    // Find next vertex with constrained yaw angle.
    Eigen::VectorXd to_yaw;
    size_t j = i + 1;
    while (j != corrected_yaw_vertices.size()) {
      if (corrected_yaw_vertices[j].getConstraint(
              mtg::derivative_order::ORIENTATION, &to_yaw)) {
        break;
      } else {
        j++;
      }
    }
    if (j == corrected_yaw_vertices.size()) {
      break;  // No consecutive yaw angle found.
    }

    // Compute shortest yaw distance.
    double shortest_angular_distance =
        angles::shortest_angular_distance(from_yaw[0], to_yaw[0]);
    double equivalent_angle = (from_yaw[0] + shortest_angular_distance);
    corrected_yaw_vertices[j].addConstraint(mtg::derivative_order::ORIENTATION,
                                            equivalent_angle);
  }

  if (!linearOptimization<kNYaw>(corrected_yaw_vertices, segment_times,
                                 derivative_to_optimize, trajectory)) {
    ROS_ERROR("Failed linear yaw optimization.");
    return false;
  }

  return true;
}

bool BaseTrajectory::linearPosAndYawOptimization(
    const mtg::Vertex::Vector& pos_vertices,
    const mtg::Vertex::Vector& yaw_vertices,
    const std::vector<double>& segment_times, int pos_derivative_to_optimize,
    int yaw_derivative_to_optimize, mtg::Trajectory* trajectory) {
  ROS_ASSERT(trajectory);
  trajectory->clear();

  mtg::Trajectory pos_trajectory;
  if (!linearOptimization<kNPos>(pos_vertices, segment_times,
                                 pos_derivative_to_optimize, &pos_trajectory)) {
    ROS_ERROR("Failed linear position optimization.");
    return false;
  }

  mtg::Trajectory yaw_trajectory;
  if (!linearYawOptimization(yaw_vertices, segment_times,
                             yaw_derivative_to_optimize, &yaw_trajectory)) {
    ROS_ERROR("Failed linear yaw optimization.");
    return false;
  }

  if (!pos_trajectory.getTrajectoryWithAppendedDimension(yaw_trajectory,
                                                         trajectory)) {
    ROS_ERROR("Failed to merge position and yaw trajectory.");
    return false;
  }

  return true;
}

bool BaseTrajectory::searchSegmentTimeLinear(
    const mtg::Vertex::Vector& pos_vertices,
    const mtg::Vertex::Vector& yaw_vertices, int pos_derivative_to_optimize,
    int yaw_derivative_to_optimize,
    const mtg::FeasibilityRecursive& feasibility, double* segment_time,
    mtg::Trajectory* best_trajectory) {
  CHECK_NOTNULL(segment_time);
  CHECK_EQ(pos_vertices.size(), yaw_vertices.size());
  CHECK_EQ(pos_vertices.size(), 2);
  double a_min = std::numeric_limits<double>::max();

  // Search for first feasible time.
  mtg::InputFeasibilityResult result =
      mtg::InputFeasibilityResult::kInputIndeterminable;
  const double kUpperBound = 20 * 60;  // 20 min.
  const int kTimesToEvaluate = 100;
  double max_time = 0.1;
  double min_time = max_time;
  const double kDeltaT = (kUpperBound - max_time) / kTimesToEvaluate;
  mtg::Trajectory trajectory;
  while (result != mtg::InputFeasibilityResult::kInputFeasible &&
         max_time < kUpperBound) {
    // Linear optimization.
    if (!linearPosAndYawOptimization(pos_vertices, yaw_vertices, {max_time},
                                     pos_derivative_to_optimize,
                                     yaw_derivative_to_optimize, &trajectory)) {
      ROS_ERROR("Linear optimization failed.");
      return false;
    }
    // Feasibility check.
    result = feasibility.checkInputFeasibilityTrajectory(trajectory);
    // Increment segment time.
    if (result != mtg::InputFeasibilityResult::kInputFeasible) {
      double v_max, a_max;
      trajectory.computeMaxVelocityAndAcceleration(&v_max, &a_max);
      if (a_max < a_min) {
        a_min = a_max;
        *segment_time = max_time;  // Best time so far.
        if (best_trajectory) *best_trajectory = trajectory;
      }
      a_min = a_max < a_min ? a_max : a_min;
      max_time += kDeltaT;
    }
  }

  // TODO(rikba): Fallback solution if this fails.
  // TODO(rikba): The linear optimization seems to have problems if the
  // trajectory becomes too "crazy".
  if (result != mtg::InputFeasibilityResult::kInputFeasible) {
    ROS_ERROR_STREAM("Maximum time trajectory is input infeasible: "
                     << mtg::getInputFeasibilityResultName(result)
                     << " maximum time candidate: " << max_time);
    ROS_ERROR("Could not find any feasible segments.");
    CHECK(mtg::sampledTrajectoryStatesToFile("/tmp/failed_trajectory.txt",
                                             trajectory));
    return false;
  } else {
    CHECK(mtg::sampledTrajectoryStatesToFile("/tmp/feasible_trajectory.txt",
                                             trajectory));

    double v_max, a_max;
    trajectory.computeMaxVelocityAndAcceleration(&v_max, &a_max);
  }

  // Binary search for smaller time.
  const double kResolution = 0.05;
  while (max_time - min_time > kResolution) {
    double mid_time = (max_time + min_time) * 0.5;

    if (!linearPosAndYawOptimization(pos_vertices, yaw_vertices, {mid_time},
                                     pos_derivative_to_optimize,
                                     yaw_derivative_to_optimize, &trajectory)) {
      ROS_ERROR("Linear optimization failed.");
      return false;
    }
    // Feasibility check.
    result = feasibility.checkInputFeasibilityTrajectory(trajectory);
    if (result == mtg::InputFeasibilityResult::kInputFeasible) {
      max_time = mid_time;
    } else {
      min_time = mid_time;
    }
  }

  *segment_time = max_time;
  if (best_trajectory) *best_trajectory = trajectory;
  return true;
}

visualization_msgs::MarkerArray BaseTrajectory::getTrajectoryMarkers(
    const std::string& ns_prefix /* = "" */) {
  if (markers_cache_.markers.empty()) {
    mav_trajectory_generation::Trajectory trajectory;
    planTrajectory(&trajectory);
    const std::string kFrameId = "ground_ENU_rtk";

    const double kSamplingTime = 0.1;
    const double kMarkerTime = 10.0;
    mav_msgs::EigenTrajectoryPoint::Vector trajectory_points;
    if (!mav_trajectory_generation::sampleWholeTrajectory(
            trajectory, kSamplingTime, &trajectory_points))
      return markers_cache_;
    mav_trajectory_generation::drawMavSampledTrajectorybyTime(
        trajectory_points, kMarkerTime, kFrameId, &markers_cache_);
  }

  // Add prefix if not already added.
  if (!ns_prefix.empty())
    for (visualization_msgs::Marker& m : markers_cache_.markers)
      if (m.ns.rfind(ns_prefix, 0) != 0) m.ns = ns_prefix + "_" + m.ns;

  return markers_cache_;
}

void BaseTrajectory::sortByStart(const Eigen::Vector3d& start) {
  size_t closest_pos_vertex = 0;
  double reward = std::numeric_limits<double>::lowest();

  for (size_t i = 0; i < position_vertices_.size(); ++i) {
    Eigen::VectorXd traj_pos, traj_vel;
    position_vertices_[i].getConstraint(mtg::derivative_order::POSITION,
                                        &traj_pos);
    position_vertices_[i].getConstraint(mtg::derivative_order::VELOCITY,
                                        &traj_vel);

    const Eigen::VectorXd direction = (traj_pos - start).normalized();
    const double temp_reward = direction.dot(traj_vel);
    if (temp_reward > reward) {
      reward = temp_reward;
      closest_pos_vertex = i;
    }
  }

  std::rotate(position_vertices_.begin(),
              position_vertices_.begin() + closest_pos_vertex,
              position_vertices_.end());

  std::rotate(yaw_vertices_.begin(), yaw_vertices_.begin() + closest_pos_vertex,
              yaw_vertices_.end());
}

bool BaseTrajectory::getConstraints(const mtg::Vertex& pos_vertex,
                                    const mtg::Vertex& yaw_vertex,
                                    Eigen::VectorXd* pos,
                                    Eigen::VectorXd* yaw) {
  CHECK_NOTNULL(pos);
  CHECK_NOTNULL(yaw);

  if (!pos_vertex.getConstraint(mtg::derivative_order::POSITION, pos)) {
    ROS_ERROR("Position not constrained.");
    return false;
  }

  if (!yaw_vertex.getConstraint(mtg::derivative_order::ORIENTATION, yaw)) {
    ROS_ERROR("Yaw not constrained.");
    return false;
  }

  return true;
}

bool BaseTrajectory::addAccelerationVertices() {
  if (!addStartAccelerationVertex()) {
    ROS_ERROR("Could not add start acceleration vertex.");
    return false;
  }

  if (!addGoalDecelerationVertex()) {
    ROS_ERROR("Could not add goal acceleration vertex.");
    return false;
  }
  return true;
}

bool BaseTrajectory::connectVertices(const mtg::Vertex& start_pos_vertex,
                                     const mtg::Vertex& start_yaw_vertex,
                                     const mtg::Vertex& goal_pos_vertex,
                                     const mtg::Vertex& goal_yaw_vertex,
                                     bool insert_begin) {
  // Find segment time.
  mtg::Vertex::Vector pos_vertices = {start_pos_vertex, goal_pos_vertex};
  mtg::Vertex::Vector yaw_vertices = {start_yaw_vertex, goal_yaw_vertex};
  double seg_time = 0.0;

  mtg::Trajectory trajectory;
  if (!searchSegmentTimeLinear(
          pos_vertices, yaw_vertices, kPosDerivativeToOptimize,
          kYawDerivativeToOptimize, settings_->feasibility_check, &seg_time,
          &trajectory)) {
    ROS_WARN("Failed to find optimal segment times. Scaling trajectory.");
    // Rescale trajectory.
    if (!scaleTrajectory(&trajectory)) return false;
    seg_time = trajectory.getMaxTime();
  }

  mtg::Vertex::Vector::const_iterator it_pos_to_insert =
      insert_begin ? position_vertices_.begin() : position_vertices_.end();
  mtg::Vertex::Vector::const_iterator it_yaw_to_insert =
      insert_begin ? yaw_vertices_.begin() : yaw_vertices_.end();
  std::vector<double>::const_iterator it_time_to_insert =
      insert_begin ? segment_times_.begin() : segment_times_.end();

  const mtg::Vertex& pos_vertex_to_insert =
      insert_begin ? start_pos_vertex : goal_pos_vertex;
  const mtg::Vertex& yaw_vertex_to_insert =
      insert_begin ? start_yaw_vertex : goal_yaw_vertex;

  position_vertices_.insert(it_pos_to_insert, pos_vertex_to_insert);
  yaw_vertices_.insert(it_yaw_to_insert, yaw_vertex_to_insert);
  segment_times_.insert(it_time_to_insert, seg_time);
  return true;
}

bool BaseTrajectory::addStartAccelerationVertex() {
  if (position_vertices_.empty() || yaw_vertices_.empty() ||
      segment_times_.empty()) {
    ROS_ERROR("Vertices not set.");
    return false;
  }

  // Get start constraint.
  Eigen::VectorXd start_pos, start_yaw;
  if (!getConstraints(position_vertices_.front(), yaw_vertices_.front(),
                      &start_pos, &start_yaw))
    return false;

  // Create vertex.
  mtg::Vertex start_pos_vertex(kPositionDimension);
  start_pos_vertex.makeStartOrEnd(start_pos, kMaxDerivativePos);
  mtg::Vertex start_yaw_vertex(kYawDimension);
  start_yaw_vertex.makeStartOrEnd(start_yaw, kMaxDerivativeYaw);

  // Create vertex.
  return connectVertices(start_pos_vertex, start_yaw_vertex,
                         position_vertices_.front(), yaw_vertices_.front(),
                         true);
}

bool BaseTrajectory::addGoalDecelerationVertex() {
  if (position_vertices_.empty() || yaw_vertices_.empty() ||
      segment_times_.empty()) {
    ROS_ERROR("Vertices not set.");
    return false;
  }

  // Get start constraint.
  Eigen::VectorXd goal_pos, goal_yaw;
  if (!getConstraints(position_vertices_.back(), yaw_vertices_.back(),
                      &goal_pos, &goal_yaw))
    return false;

  // Create vertex.
  mtg::Vertex goal_pos_vertex(kPositionDimension);
  goal_pos_vertex.makeStartOrEnd(goal_pos, kMaxDerivativePos);
  mtg::Vertex goal_yaw_vertex(kYawDimension);
  goal_yaw_vertex.makeStartOrEnd(goal_yaw, kMaxDerivativeYaw);

  // Create vertex.
  return connectVertices(position_vertices_.back(), yaw_vertices_.back(),
                         goal_pos_vertex, goal_yaw_vertex, false);
}

bool BaseTrajectory::connectToPrevious() {
  auto prev_trajectory = settings_->prev_trajectory.lock();
  if (!prev_trajectory) return true;  // Nothing to do here.
  if (position_vertices_.empty() || yaw_vertices_.empty() ||
      segment_times_.empty()) {
    ROS_ERROR("Vertices not set.");
    return false;
  }
  mtg::Vertex prev_pos(kPositionDimension), prev_yaw(kYawDimension);
  prev_trajectory->getGoalVertex(&prev_pos, &prev_yaw);
  return connectVertices(prev_pos, prev_yaw, position_vertices_.front(),
                         yaw_vertices_.front(), true);
}

bool BaseTrajectory::scaleTrajectory(mtg::Trajectory* trajectory,
                                     double* scaling) const {
  CHECK_NOTNULL(trajectory);

  // Find first feasible trajectory.
  mtg::InputFeasibilityResult result =
      settings_->feasibility_check.checkInputFeasibilityTrajectory(*trajectory);
  double max_scaling = 1.0;
  int max_iterations = 20;
  int iteration = 0;

  while (result != mtg::InputFeasibilityResult::kInputFeasible) {
    max_scaling *= 2.0;
    mtg::Trajectory temp_trajectory = *trajectory;
    if (!temp_trajectory.scaleSegmentTimes(max_scaling)) {
      ROS_ERROR("Failed to scale trajectory.");
      return false;
    }
    result = settings_->feasibility_check.checkInputFeasibilityTrajectory(
        temp_trajectory);

    if (++iteration == max_iterations) {
      ROS_ERROR_STREAM("Failed to find upscaled feasible trajectory after "
                       << iteration << " iterations.");
      return false;
    }
  }

  // TODO(rikba): Binary search for better scaling.
  const double kScaleResolution = 0.1;
  double min_scaling = 1.0;

  while (max_scaling - min_scaling > kScaleResolution) {
    double mid_scaling = (max_scaling + min_scaling) * 0.5;
    mtg::Trajectory temp_trajectory = *trajectory;
    if (!temp_trajectory.scaleSegmentTimes(mid_scaling)) {
      ROS_ERROR("Failed to scale trajectory during binary search.");
      return false;
    }
    if (settings_->feasibility_check.checkInputFeasibilityTrajectory(
            temp_trajectory) == mtg::InputFeasibilityResult::kInputFeasible) {
      max_scaling = mid_scaling;
    } else {
      min_scaling = mid_scaling;
    }
  }

  ROS_WARN_STREAM("Scaling trajectory by: " << max_scaling);
  trajectory->scaleSegmentTimes(max_scaling);

  if (scaling) *scaling = max_scaling;

  return true;
}

void BaseTrajectory::getVertices(
    mtg::Vertex::Vector* pos_vertices, mtg::Vertex::Vector* yaw_vertices,
    std::vector<double>* segment_times /* = nullptr */) const {
  CHECK_NOTNULL(pos_vertices);
  CHECK_NOTNULL(yaw_vertices);

  *pos_vertices = position_vertices_;
  *yaw_vertices = yaw_vertices_;
  if (segment_times) *segment_times = segment_times_;
}

void BaseTrajectory::getStartVertex(mtg::Vertex* start_pos_vertex,
                                    mtg::Vertex* start_yaw_vertex) const {
  getStartPosVertex(start_pos_vertex);
  getStartYawVertex(start_yaw_vertex);
}

void BaseTrajectory::getGoalVertex(mtg::Vertex* goal_pos_vertex,
                                   mtg::Vertex* goal_yaw_vertex) const {
  getGoalPosVertex(goal_pos_vertex);
  getGoalYawVertex(goal_yaw_vertex);
}

void BaseTrajectory::getStartPosVertex(mtg::Vertex* start_pos_vertex) const {
  CHECK_NOTNULL(start_pos_vertex);
  if (position_vertices_.empty()) return;
  *start_pos_vertex = position_vertices_.front();
}

void BaseTrajectory::getGoalPosVertex(mtg::Vertex* goal_pos_vertex) const {
  CHECK_NOTNULL(goal_pos_vertex);
  if (position_vertices_.empty()) return;
  *goal_pos_vertex = position_vertices_.back();
}

void BaseTrajectory::getStartYawVertex(mtg::Vertex* start_yaw_vertex) const {
  CHECK_NOTNULL(start_yaw_vertex);
  if (yaw_vertices_.empty()) return;
  *start_yaw_vertex = yaw_vertices_.front();
}

void BaseTrajectory::getGoalYawVertex(mtg::Vertex* goal_yaw_vertex) const {
  CHECK_NOTNULL(goal_yaw_vertex);
  if (yaw_vertices_.empty()) return;
  *goal_yaw_vertex = yaw_vertices_.back();
}

bool BaseTrajectory::toYaml(YAML::Node* node) {
  CHECK_NOTNULL(node);
  return true;
}
void BaseTrajectory::samplePositionVertices() {}
void BaseTrajectory::sampleYawVertices() {}
void BaseTrajectory::sampleTimes() {}

bool BaseTrajectory::enterTrajectory(const mtg::Vertex& start_pos_vertex,
                                     const mtg::Vertex& start_yaw_vertex) {
  return connectVertices(start_pos_vertex, start_yaw_vertex,
                         position_vertices_.front(), yaw_vertices_.front(),
                         true);
}

bool BaseTrajectory::exitTrajectory(const mtg::Vertex& goal_pos_vertex,
                                    const mtg::Vertex& goal_yaw_vertex) {
  return connectVertices(position_vertices_.back(), yaw_vertices_.back(),
                         goal_pos_vertex, goal_yaw_vertex, false);
}

}  // namespace fm_trajectories
