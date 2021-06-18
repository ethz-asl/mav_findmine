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

#include "fm_trajectories/trajectory_generator.h"

#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Point.h>
#include <mav_msgs/default_values.h>
#include <mav_trajectory_generation/io.h>
#include <mav_trajectory_generation/motion_defines.h>
#include <mav_trajectory_generation/trajectory.h>
#include <mav_trajectory_generation/vertex.h>
#include <mav_trajectory_generation_ros/input_constraints.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <visualization_msgs/MarkerArray.h>

#include "fm_trajectories/base_trajectory.h"

namespace mtg = mav_trajectory_generation;

namespace fm_trajectories {

TrajectoryGenerator::TrajectoryGenerator(const ros::NodeHandle& nh,
                                         const ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private) {
  advertiseServices();
  advertiseTopics();

  ROS_ERROR_COND(
      !geotf_.addFrameByEPSG("wgs84", 4326),
      "Failed to add frame ecef as EPSG:4326. Be careful using converted "
      "positions");
}

void TrajectoryGenerator::advertiseServices() {
  trajectory_srv_ = nh_private_.advertiseService(
      "generate_trajectory", &TrajectoryGenerator::generateTrajectory, this);
}

void TrajectoryGenerator::advertiseTopics() {
  const bool kLatchTopic = true;
  const bool kQueueSize = 1;
  trajectory_marker_pub_ =
      nh_private_.advertise<visualization_msgs::MarkerArray>(
          "trajectory_marker", kQueueSize, kLatchTopic);
}

bool TrajectoryGenerator::generateTrajectory(
    fm_comm::TrajectoryService::Request& req,
    fm_comm::TrajectoryService::Response& res) {
  // Remove old trajectory markers.
  if (base_trajectory_) {
    trajectory_marker_pub_.publish(
        base_trajectory_->getTrajectoryDeleteMarkers());
  }

  // Open mission file.
  YAML::Node mission = YAML::LoadFile(req.mission_file.data);

  // Load geodetic reference from file.
  if (!geodeticReferenceFromYaml(mission["geodetic_reference"])) {
    ROS_ERROR("Cannot load geodetic reference from mission.");
    return false;
  }

  // Transform input trajectory from original ENU frame A to target ENU frame B.
  // Translation from trajectory ENU origin to new trajectory ENU origin.
  Eigen::Vector3d A_r_B;
  Eigen::Vector3d A_r_B_wgs84(req.target_lat_ENU.data, req.target_lon_ENU.data,
                              req.target_alt_ENU.data);
  if (!geotf_.convert("wgs84", A_r_B_wgs84, "enu", &A_r_B)) {
    ROS_ERROR("Failed to convert WGS84 to ENU.");
    return false;
  }
  A_r_B.z() = 0.0;  // Ignore change in altitude for now.

  mtg::Trajectory trajectory;  // Trajectory in original frame A.
  if (!mtg::trajectoryFromYaml(mission["trajectory"], &trajectory)) {
    ROS_ERROR("Cannot load trajectory from mission.");
    return false;
  }

  // We do a frame transformation from frame A to frame B, hence the negative
  // sign.
  if (!trajectory.offsetTrajectory(-A_r_B)) return false;

  // Create base trajectory object.
  const double kDummyVelocity = 0.1;
  const double kOffsetHeading = 0.0;
  const std::weak_ptr<BaseTrajectory> kPrevTrajectory;
  const double kConstantVelocity = false;

  mtg::InputConstraints input_constraints;
  input_constraints.fromYaml(mission["input_constraints"]);

  std::shared_ptr<BaseTrajectory::Settings> settings =
      std::make_shared<BaseTrajectory::Settings>(
          input_constraints, kDummyVelocity, kOffsetHeading, kPrevTrajectory,
          kConstantVelocity);
  base_trajectory_ =
      std::unique_ptr<BaseTrajectory>(new BaseTrajectory(trajectory, settings));

  if (req.plan_from_home.data) {
    ROS_INFO("expanding trajectory by start and goal pose!");

    // Plan to start.
    mtg::Vertex start_pos(kPositionDimension), start_yaw(kYawDimension);
    vertexFromPoseTwist(req.start_pose, req.start_twist, &start_pos,
                        &start_yaw);
    if (!base_trajectory_->enterTrajectory(start_pos, start_yaw)) return false;

    // Plan to goal.
    mtg::Vertex goal_pos(kPositionDimension), goal_yaw(kYawDimension);
    vertexFromPoseTwist(req.goal_pose, req.goal_twist, &goal_pos, &goal_yaw);
    if (!base_trajectory_->exitTrajectory(goal_pos, goal_yaw)) return false;
  }

  // Get trajectory.
  mtg::Trajectory response_trajectory;
  const bool kUseCache = false;
  if (!base_trajectory_->planTrajectory(&response_trajectory, kUseCache))
    return false;

  if (!mtg::trajectoryToPolynomialTrajectoryMsg(response_trajectory,
                                                &res.trajectory))
    return false;

  mtg::Vertex trajectory_start_vertex(kPositionDimension);
  trajectory_start_vertex = response_trajectory.getStartVertex(0);

  Eigen::VectorXd start_pos_;
  geometry_msgs::Point start_pos_msg;

  trajectory_start_vertex.getConstraint(
      0, &start_pos_);  // get position constraint!
  tf::pointEigenToMsg(start_pos_, start_pos_msg);

  res.start_pos = start_pos_msg;

  ROS_INFO(
      "fm_trajectories: trajectory_generator trajectory and trajectory start "
      "assigned");

  // Publish trajectory markers.
  trajectory_marker_pub_.publish(base_trajectory_->getTrajectoryMarkers());

  return true;
}

void TrajectoryGenerator::vertexFromPoseTwist(const geometry_msgs::Pose& pose,
                                              const geometry_msgs::Twist& twist,
                                              mtg::Vertex* pos_vertex,
                                              mtg::Vertex* yaw_vertex) const {
  CHECK_NOTNULL(pos_vertex);
  CHECK_NOTNULL(yaw_vertex);

  Eigen::Vector3d position_W, velocity_W;
  tf::pointMsgToEigen(pose.position, position_W);
  tf::vectorMsgToEigen(twist.linear, velocity_W);

  Eigen::Quaterniond orientation_W_B;
  tf::quaternionMsgToEigen(pose.orientation, orientation_W_B);
  double yaw = mav_msgs::yawFromQuaternion(orientation_W_B);

  Eigen::Vector3d angular_velocity_B;
  tf::vectorMsgToEigen(twist.angular, angular_velocity_B);
  Eigen::Vector3d angular_velocity_W =
      orientation_W_B.toRotationMatrix() * angular_velocity_B;
  double yaw_rate = angular_velocity_W.z();

  *pos_vertex = mtg::Vertex(kPositionDimension);
  pos_vertex->makeStartOrEnd(position_W, kMaxDerivativePos);
  pos_vertex->addConstraint(mtg::derivative_order::VELOCITY, velocity_W);

  *yaw_vertex = mtg::Vertex(kYawDimension);
  yaw_vertex->makeStartOrEnd(yaw, kMaxDerivativeYaw);
  yaw_vertex->addConstraint(mtg::derivative_order::ANGULAR_VELOCITY, yaw_rate);
}

bool TrajectoryGenerator::geodeticReferenceFromYaml(const YAML::Node& node) {
  if (!node["lat_enu"]) return false;
  if (!node["lon_enu"]) return false;
  if (!node["alt_enu"]) return false;

  geotf_.removeFrame("enu");
  geotf_.addFrameByENUOrigin("enu", node["lat_enu"].as<double>(),
                             node["lon_enu"].as<double>(),
                             node["alt_enu"].as<double>());
  return true;
}

}  // namespace fm_trajectories
