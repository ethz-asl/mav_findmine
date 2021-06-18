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

#ifndef FM_TRAJECTORIES_BASE_TRAJECTORY_H_
#define FM_TRAJECTORIES_BASE_TRAJECTORY_H_

#include <geotf/geodetic_converter.h>
#include <mav_msgs/default_values.h>
#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation/trajectory.h>
#include <mav_trajectory_generation_ros/feasibility_recursive.h>
#include <mav_trajectory_generation_ros/input_constraints.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <yaml-cpp/yaml.h>
#include <cmath>
#include <memory>

namespace fm_trajectories {

namespace mtg = mav_trajectory_generation;

enum TrajectoryType {
  Hotpoint = 0,
  Rectangle,
  Polygon,
  Strip,
  Connection,
  Waypoints
};

constexpr int kNPos = 10;
constexpr int kNYaw = 6;
constexpr int kMaxDerivativePos = kNPos / 2 - 1;
constexpr int kMaxDerivativeYaw = kNYaw / 2 - 1;
const std::vector<size_t> kPositionIndices = {0, 1, 2};
const std::vector<size_t> kYawIndices = {3};
const size_t kPositionDimension = kPositionIndices.size();
const size_t kYawDimension = kYawIndices.size();
const int kPosDerivativeToOptimize = mtg::derivative_order::JERK;
const int kYawDerivativeToOptimize =
    mtg::derivative_order::ANGULAR_ACCELERATION;
const double kZeroGuard = 1.0e-3;
// TODO(rikba): Fix numeric issues for very small velocities.
const double kMinVelocity = 0.1;          // [m/s]
const double kVelocityResolution = 0.01;  // [m/s]

class BaseTrajectory {
 public:
  struct Settings {
    Settings(const mtg::InputConstraints& input_constraints, double velocity,
             double offset_heading_deg,
             const std::weak_ptr<BaseTrajectory> prev_trajectory,
             bool constant_velocity = true);
    mtg::FeasibilityRecursive feasibility_check;
    double velocity;
    // Heading offset with respect to flight direction.
    double offset_heading_deg;
    std::weak_ptr<BaseTrajectory> prev_trajectory;
    bool constant_velocity;  // Plan are start and stop maneuver to achieve
                             // constant velocity on the trajectory.
  };

  BaseTrajectory(const mtg::Trajectory& trajectory,
                 const std::shared_ptr<Settings>& settings);
  inline BaseTrajectory(const std::shared_ptr<Settings>& settings)
      : settings_(settings) {
    ROS_ERROR_COND(
        !geotf_.addFrameByEPSG("wgs84", 4326),
        "Failed to add frame ecef as EPSG:4326. Be careful using converted "
        "positions");
  }
  void setGeodeticReference(double lat, double lon, double alt);

  virtual bool planTrajectory(mtg::Trajectory* trajectory,
                              bool use_cache = true);

  // TODO(rikba): Make const methods.
  virtual bool toYaml(YAML::Node* node);
  visualization_msgs::MarkerArray getTrajectoryMarkers(
      const std::string& ns_prefix = "");
  visualization_msgs::MarkerArray getTrajectoryDeleteMarkers();

  // Static optimization helpers.
  // Wrapper for linear optimization.
  template <int N>
  static inline bool linearOptimization(
      const mtg::Vertex::Vector& vertices,
      const std::vector<double>& segment_times, int derivative_to_optimize,
      mtg::Trajectory* trajectory) {
    CHECK_NOTNULL(trajectory);
    trajectory->clear();

    mtg::PolynomialOptimization<N> opt(vertices.front().D());
    if (!opt.setupFromVertices(vertices, segment_times,
                               derivative_to_optimize)) {
      ROS_ERROR("Failed to setup linear optimization.");
      return false;
    }
    if (!opt.solveLinear()) {
      ROS_ERROR("Failed to solve linear optimization.");
      return false;
    }

    opt.getTrajectory(trajectory);

    return true;
  }

  // Wrapper for nonlinear optimization.
  template <int N>
  static inline bool nonlinearOptimization(
      const mtg::Vertex::Vector& vertices, int derivative_to_optimize,
      const mtg::InputConstraints& constraints, mtg::Trajectory* trajectory) {
    CHECK_NOTNULL(trajectory);
    trajectory->clear();

    // Get constraints.
    double v_max, a_max;
    double f_min = std::numeric_limits<double>::max();
    double f_max = std::numeric_limits<double>::max();
    if (!constraints.getConstraint(mtg::InputConstraintType::kVMax, &v_max)) {
      ROS_ERROR("Velocity not constrained.");
      return false;
    } else if (!constraints.getConstraint(mtg::InputConstraintType::kFMin,
                                          &f_min) ||
               !constraints.getConstraint(mtg::InputConstraintType::kFMax,
                                          &f_max)) {
      ROS_ERROR("Thrust not constrained.");
      return false;
    }
    a_max = std::min(std::fabs(f_min - mav_msgs::kGravity),
                     std::fabs(f_max - mav_msgs::kGravity));

    // Initial guess segment times.
    std::cout << "Estimate. " << f_min << " " << f_max << " " << v_max << " "
              << a_max << std::endl;
    std::vector<double> segment_times =
        mav_trajectory_generation::estimateSegmentTimesVelocityRamp(
            vertices, v_max, a_max, 1.0);

    std::cout << "Adjust." << std::endl;
    for (double& t : segment_times)
      if (t < kZeroGuard) t = 1.0;  // Hacky initial time guess.

    // Nonlinear optimization.
    mav_trajectory_generation::NonlinearOptimizationParameters nlopt_parameters;
    nlopt_parameters.algorithm = nlopt::LD_LBFGS;
    nlopt_parameters.time_alloc_method = mav_trajectory_generation::
        NonlinearOptimizationParameters::kMellingerOuterLoop;
    nlopt_parameters.print_debug_info_time_allocation = true;
    mav_trajectory_generation::PolynomialOptimizationNonLinear<N> nlopt(
        kPositionDimension, nlopt_parameters);
    nlopt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
    nlopt.addMaximumMagnitudeConstraint(
        mav_trajectory_generation::derivative_order::VELOCITY, v_max);
    nlopt.addMaximumMagnitudeConstraint(
        mav_trajectory_generation::derivative_order::ACCELERATION, a_max);
    nlopt.optimize();
    nlopt.getTrajectory(trajectory);

    return true;
  }

  // Wrapper for yaw optimization. Takes into account yaw wrap around.
  static bool linearYawOptimization(const mtg::Vertex::Vector& yaw_vertices,
                                    const std::vector<double>& segment_times,
                                    int derivative_to_optimize,
                                    mtg::Trajectory* trajectory);
  // Wrapper for both position and yaw optimization.
  static bool linearPosAndYawOptimization(
      const mtg::Vertex::Vector& pos_vertices,
      const mtg::Vertex::Vector& yaw_vertices,
      const std::vector<double>& segment_times, int pos_derivative_to_optimize,
      int yaw_derivative_to_optimize, mtg::Trajectory* trajectory);

  // Brute force searches different segment times and picks the timewise
  // shortest feasible trajectory.
  static bool searchSegmentTimeLinear(
      const mtg::Vertex::Vector& pos_vertices,
      const mtg::Vertex::Vector& yaw_vertices, int pos_derivative_to_optimize,
      int yaw_derivative_to_optimize,
      const mtg::FeasibilityRecursive& feasibility, double* segment_time,
      mtg::Trajectory* best_trajectory = nullptr);

  bool scaleTrajectory(mtg::Trajectory* trajectory,
                       double* scaling = nullptr) const;

  // Find the vertex in the trajectory that has a similar velocity direction as
  // the straight line connecting the start with the trajectory. Circulate
  // vertices.
  void sortByStart(const Eigen::Vector3d& start);

  // Turn into flight direction, fly straight line, turn.
  bool enterTrajectory(const mtg::Vertex& start_pos_vertex,
                       const mtg::Vertex& start_yaw_vertex);
  bool exitTrajectory(const mtg::Vertex& goal_pos_vertex,
                      const mtg::Vertex& goal_yaw_vertex);

  void getVertices(mtg::Vertex::Vector* pos_vertices,
                   mtg::Vertex::Vector* yaw_vertices,
                   std::vector<double>* segment_times = nullptr) const;

  void getStartVertex(mtg::Vertex* start_pos_vertex,
                      mtg::Vertex* start_yaw_vertex) const;

  void getGoalVertex(mtg::Vertex* goal_pos_vertex,
                     mtg::Vertex* goal_yaw_vertex) const;

  void getStartPosVertex(mtg::Vertex* start_pos_vertex) const;
  void getGoalPosVertex(mtg::Vertex* goal_pos_vertex) const;
  void getStartYawVertex(mtg::Vertex* start_yaw_vertex) const;
  void getGoalYawVertex(mtg::Vertex* goal_yaw_vertex) const;

  double getTime() const { return trajectory_cache_.getMaxTime(); }

 protected:
  virtual void samplePositionVertices();
  virtual void sampleYawVertices();
  virtual void sampleTimes();
  bool sampleVertices();

  virtual bool addAccelerationVertices();
  bool connectVertices(const mtg::Vertex& start_pos_vertex,
                       const mtg::Vertex& start_yaw_vertex,
                       const mtg::Vertex& goal_pos_vertex,
                       const mtg::Vertex& goal_yaw_vertex, bool insert_begin);
  bool addStartAccelerationVertex();
  bool addGoalDecelerationVertex();

  bool connectToPrevious();

  mtg::Vertex::Vector position_vertices_;
  mtg::Vertex::Vector yaw_vertices_;
  std::vector<double> segment_times_;

  geotf::GeodeticConverter geotf_;
  mav_trajectory_generation::Trajectory trajectory_cache_;
  visualization_msgs::MarkerArray markers_cache_;

  std::shared_ptr<Settings> settings_;

 private:
  bool computeTrajectory(mtg::Trajectory* trajectory);
  bool getConstraints(const mtg::Vertex& pos_vertex,
                      const mtg::Vertex& yaw_vertex, Eigen::VectorXd* pos,
                      Eigen::VectorXd* yaw);
};

}  // namespace fm_trajectories

#endif  // FM_TRAJECTORIES_BASE_TRAJECTORY_H_
