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

#include "fm_control/position_controller.h"

#include <functional>
#include <limits>

#include <angles/angles.h>
#include <dji_sdk/SDKControlAuthority.h>
#include <dji_sdk/dji_sdk.h>
#include <eigen_conversions/eigen_msg.h>
#include <fm_comm/dji_interface.h>
#include <geometry_msgs/PoseStamped.h>
#include <mav_msgs/conversions.h>
#include <mav_trajectory_generation/trajectory_sampling.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>

namespace fm_control {

constexpr double kDefaultKPx = 1.0;
constexpr double kDefaultKPy = 1.0;
constexpr double kDefaultKPz = 1.0;
constexpr double kDefaultKYaw = 1.0;

constexpr double kDefaultKVx = 1.5;
constexpr double kDefaultKVy = 1.5;
constexpr double kDefaultKVz = 1.5;
constexpr double kDefaultKYawRate = 0.5;

constexpr double kDefaultKAx = 0.1;
constexpr double kDefaultKAy = 0.1;
constexpr double kDefaultKAz = 0.1;
constexpr double kDefaultKYawAcc = 0.0;

constexpr double kDefaultVXyMax = 12.0;
constexpr double kDefaultVZUpMax = 3.0;
constexpr double kDefaultVZDownMax = 2.0;
const double kDefaultYawRateMax = angles::from_degrees(120.0);

const ControlType kDefaultControlType = ControlType::kVelocity;
const AltitudeMode kDefaultAltitudeMode = AltitudeMode::kENU;

constexpr double kDefaultMinTransitionAltitude = -1.0;
PositionController::PositionController(const ros::NodeHandle& nh,
                                       const ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private) {
  advertiseServices();
  subscribeServices();
  advertiseTopics();
  subscribeTopics();
  intializeDynamicReconfigure();
}

PositionController::States::States()
    : has_ctrl_authority(false),
      goal_type(GoalType::kNone),
      has_position(false),
      has_velocity(false),
      has_translational_acceleration(false),
      has_attitude(false),
      has_angular_velocity(false),
      has_angular_acceleration(false),
      has_agl(false),
      flight_mode_axis(std::numeric_limits<int>::max()) {}

void PositionController::States::reset() { *this = States(); }

PositionController::Parameters::Parameters()
    : K_px(kDefaultKPx),
      K_py(kDefaultKPy),
      K_pz(kDefaultKPz),
      K_yaw(kDefaultKYaw),
      K_vx(kDefaultKVx),
      K_vy(kDefaultKVy),
      K_vz(kDefaultKVz),
      K_yaw_rate(kDefaultKYawRate),
      K_ax(kDefaultKAx),
      K_ay(kDefaultKAy),
      K_az(kDefaultKAz),
      K_yaw_acc(kDefaultKYawAcc),
      v_xy_max(kDefaultVXyMax),
      v_z_up_max(kDefaultVZUpMax),
      v_z_down_max(kDefaultVZDownMax),
      yaw_rate_max(kDefaultYawRateMax),
      control_type(kDefaultControlType),
      altitude_mode(kDefaultAltitudeMode),
      min_transition_altitude(kDefaultMinTransitionAltitude) {}

void PositionController::intializeDynamicReconfigure() {
  dynamic_reconfigure::Server<PositionControllerConfig>::CallbackType f;
  f = std::bind(&PositionController::dynamicReconfigure, this,
                std::placeholders::_1, std::placeholders::_2);
  dyn_config_server_.setCallback(f);
}

void PositionController::dynamicReconfigure(PositionControllerConfig& config,
                                            uint32_t level) {
  params_.K_px = config.K_px;
  params_.K_py = config.K_py;
  params_.K_pz = config.K_pz;
  params_.K_yaw = config.K_yaw;

  params_.K_vx = config.K_vx;
  params_.K_vy = config.K_vy;
  params_.K_vz = config.K_vz;
  params_.K_yaw_rate = config.K_yaw_rate;

  params_.K_ax = config.K_ax;
  params_.K_ay = config.K_ay;
  params_.K_az = config.K_az;
  params_.K_yaw_acc = config.K_yaw_acc;

  params_.v_xy_max = config.v_xy_max;
  params_.v_z_up_max = config.v_z_up_max;
  params_.v_z_down_max = config.v_z_down_max;
  params_.yaw_rate_max = config.yaw_rate_max;

  params_.control_type = static_cast<ControlType>(config.control_type);

  params_.min_transition_altitude = config.min_transition_altitude;
}

void PositionController::advertiseServices() {
  switch_control_frame_srv_ = nh_.advertiseService(
      "switch_control_frame", &PositionController::switchControlFrame, this);
  obtain_ctrl_authority_srv_ =
      nh_.advertiseService("obtain_control_authority",
                           &PositionController::obtainControlAuthority, this);
  release_ctrl_authority_srv_ =
      nh_.advertiseService("release_control_authority",
                           &PositionController::releaseControlAuthority, this);
}

void PositionController::subscribeServices() {
  sdk_ctrl_authority_client_ = nh_.serviceClient<dji_sdk::SDKControlAuthority>(
      "dji_sdk/sdk_control_authority");
}

const bool kQueueSize = 1;
void PositionController::advertiseTopics() {
  const bool kLatchTopic = false;
  ctrl_vel_yawrate_pub_ = nh_.advertise<sensor_msgs::Joy>(
      "dji_sdk/flight_control_setpoint_ENUvelocity_yawrate", kQueueSize,
      kLatchTopic);
  ctrl_pos_yaw_pub_ = nh_.advertise<sensor_msgs::Joy>(
      "dji_sdk/flight_control_setpoint_ENUposition_yaw", kQueueSize,
      kLatchTopic);
  pos_error_pub_ = nh_private_.advertise<std_msgs::Float32>(
      "pos_error", kQueueSize, kLatchTopic);
  pos_yaw_ref_pub_ = nh_private_.advertise<geometry_msgs::PoseStamped>(
      "pos_yaw_ref", kQueueSize, kLatchTopic);
  ctrl_auth_pub_ = nh_private_.advertise<std_msgs::Bool>("ctrl_authority",
                                                         kQueueSize, false);
}

void PositionController::subscribeTopics() {
  // Translational control.
  local_position_sub_ =
      nh_.subscribe("dji_sdk/local_position", kQueueSize,
                    &PositionController::receiveLocalPosition, this);
  velocity_sub_ = nh_.subscribe("dji_sdk/velocity", kQueueSize,
                                &PositionController::receiveVelocity, this);
  acceleration_sub_ =
      nh_.subscribe("dji_sdk/acceleration_ground_fused", kQueueSize,
                    &PositionController::receiveAcceleration, this);

  // Yaw control.
  attitude_sub_ = nh_.subscribe("dji_sdk/attitude", kQueueSize,
                                &PositionController::receiveAttitude, this);
  angular_velocity_sub_ =
      nh_.subscribe("dji_sdk/angular_velocity_fused", kQueueSize,
                    &PositionController::receiveAngularVelocity, this);

  // AGL.
  agl_sub_ = nh_.subscribe("fm_altitude_estimator/state", kQueueSize,
                           &PositionController::receiveAGL, this);

  // Command subscriber.
  goal_sub_ = nh_.subscribe("command/pose", kQueueSize,
                            &PositionController::receiveGoal, this);
  poly_trajectory_sub_ =
      nh_.subscribe("command/polynomial_trajectory", kQueueSize,
                    &PositionController::receivePolynomialTrajectory, this);
  rc_sub_ = nh_.subscribe("dji_sdk/rc", kQueueSize,
                          &PositionController::receiveRC, this);
}

bool PositionController::obtainControlAuthority(
    std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
  dji_sdk::SDKControlAuthority::Request sdk_ca_req;
  sdk_ca_req.control_enable =
      dji_sdk::SDKControlAuthority::Request::REQUEST_CONTROL;
  states_.has_ctrl_authority =
      fm_comm::callDjiTask<dji_sdk::SDKControlAuthority>(
          sdk_ca_req, sdk_ctrl_authority_client_);
  if (states_.has_ctrl_authority) {
    ROS_INFO("Obtained control authority.");
  } else {
    ROS_WARN("Failed obtaining control authority.");
  }

  std_msgs::Bool msg;
  msg.data = states_.has_ctrl_authority;
  ctrl_auth_pub_.publish(msg);

  return states_.has_ctrl_authority;
}

bool PositionController::releaseControlAuthority(
    std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
  dji_sdk::SDKControlAuthority::Request sdk_ca_req;
  sdk_ca_req.control_enable =
      dji_sdk::SDKControlAuthority::Request::RELEASE_CONTROL;
  const bool success = fm_comm::callDjiTask<dji_sdk::SDKControlAuthority>(
      sdk_ca_req, sdk_ctrl_authority_client_);
  if (success && states_.has_ctrl_authority) {
    ROS_INFO("Released control authority.");
    states_.reset();  // Invalidate state.
  } else if (!success && states_.has_ctrl_authority) {
    ROS_ERROR("Cannot release control authority.");
    // Invalidate goal. Controller will stop publishing and DJI will hover at
    // current position.
    states_.reset();  // Invalidate state.
    states_.has_ctrl_authority = true;
  } else {
    states_.reset();  // Invalidate state.
  }

  std_msgs::Bool msg;
  msg.data = states_.has_ctrl_authority;
  ctrl_auth_pub_.publish(msg);

  return success;
}

bool PositionController::switchControlFrame(
    std_srvs::SetBool::Request& request,
    std_srvs::SetBool::Response& response) {
  // true means AGL!
  if (request.data && params_.altitude_mode != AltitudeMode::kAGL) {
    // check if requirements are met for AGL
    if (states_.has_agl) {
      states_.goal_type = GoalType::kPose;
      mav_msgs::EigenTrajectoryPoint new_goal;
      new_goal.timestamp_ns = ros::Time::now().toNSec();
      new_goal.orientation_W_B = states_.mav.orientation_W_B;
      new_goal.position_W = states_.mav.position_W;
      new_goal.position_W.z() = states_.agl;  // set z- setpoint to current AGL.
      states_.goal = new_goal;
      params_.altitude_mode = AltitudeMode::kAGL;
      response.success = true;
    } else {
      ROS_WARN_STREAM("Switch to AGL Mode failed, agl data not received.");
      response.success = false;
    }

  } else if (!request.data && params_.altitude_mode != kENU) {
    // Switch back to ENU
    if (states_.agl >= params_.min_transition_altitude) {
      states_.goal_type = GoalType::kPose;
      mav_msgs::EigenTrajectoryPoint new_goal;
      new_goal.timestamp_ns = ros::Time::now().toNSec();
      new_goal.orientation_W_B = states_.mav.orientation_W_B;
      new_goal.position_W = states_.mav.position_W;
      states_.goal = new_goal;
      params_.altitude_mode = AltitudeMode::kENU;
      response.success = true;
    } else {
      ROS_WARN_STREAM("Below transition altitude for AGL => ENU switch.");
      response.success = false;
    }

  } else if (request.data && params_.altitude_mode == kAGL) {
    // Already in this mode.
    response.success = true;
  } else if (!request.data && params_.altitude_mode == kENU) {
    // Already in this mode.
    response.success = true;
  }
  return true;
}

void PositionController::receiveGoal(
    const geometry_msgs::PoseStampedConstPtr& msg) {
  ROS_INFO_ONCE("Received first pose command.");
  if (!states_.has_ctrl_authority) return;

  // if (params_.altitude_mode == AltitudeMode::kAGL
  //     && msg->header.frame_id != "AGL") {
  //   ROS_WARN_STREAM(
  //       "Received Goal in frame " << msg->header.frame_id << " in AGL
  //       Mode!");
  //   return;
  // }
  //
  // if (params_.altitude_mode == AltitudeMode::kENU
  //     && msg->header.frame_id == "AGL") {
  //   ROS_WARN_STREAM(
  //       "Received Goal in frame " << msg->header.frame_id << " in ENU
  //       Mode!");
  //   return;
  // }
  mav_msgs::eigenTrajectoryPointFromPoseMsg(*msg, &states_.goal);
  states_.goal_type = GoalType::kPose;
}

void PositionController::receivePolynomialTrajectory(
    const mav_planning_msgs::PolynomialTrajectory4D& msg) {
  ROS_INFO_ONCE("Received first trajectory command.");
  if (!states_.has_ctrl_authority) return;

  // TODO(rikba): Set correct frame in trajectory message.
  // if (params_.altitude_mode == AltitudeMode::kAGL
  //     && msg.header.frame_id != "AGL") {
  //   return;
  // }
  //
  // if (params_.altitude_mode == AltitudeMode::kENU
  //     && msg.header.frame_id == "AGL") {
  //   return;
  // }

  if (!mav_trajectory_generation::polynomialTrajectoryMsgToTrajectory(
          msg, &states_.trajectory)) {
    ROS_ERROR("Cannot convert trajectory message.");
    states_.goal_type = GoalType::kNone;
    return;
  }
  states_.trajectory_start = ros::Time::now();
  states_.goal_type = GoalType::kPolynomialTrajectory;
}

void PositionController::updateGoalPose() {
  switch (states_.goal_type) {
    case GoalType::kPose: {
      break;
    }
    case GoalType::kPolynomialTrajectory: {
      double t = (ros::Time::now() - states_.trajectory_start).toSec();
      t = t > states_.trajectory.getMaxTime() ? states_.trajectory.getMaxTime()
                                              : t;
      if (!mav_trajectory_generation::sampleTrajectoryAtTime(
              states_.trajectory, t, &states_.goal)) {
        ROS_ERROR("Cannot sample trajectory. Release control authority.");
        std_srvs::Empty::Request req;
        std_srvs::Empty::Response res;
        while (!releaseControlAuthority(req, res)) {
          ros::Duration(0.1).sleep();
        }
      }
      break;
    }
    default:
      break;
  }
}

void PositionController::receiveLocalPosition(
    const geometry_msgs::PointStampedConstPtr& msg) {
  if (states_.goal_type == GoalType::kNone || !states_.has_ctrl_authority)
    return;

  states_.has_position = true;
  tf::pointMsgToEigen(msg->point, states_.mav.position_W);

  // Control action.
  // TODO(rikba): Handle message time synchronization...
  updateGoalPose();
  switch (params_.control_type) {
    case ControlType::kPosition: {
      sensor_msgs::Joy ctrl_ENUposition_yaw;
      if (updatePositionControlReference(&ctrl_ENUposition_yaw)) {
        ctrl_pos_yaw_pub_.publish(ctrl_ENUposition_yaw);
      }
      break;
    }
    case ControlType::kVelocity: {
      sensor_msgs::Joy ctrl_ENUvelocity_yawrate;
      if (updateVelocityControlReference(&ctrl_ENUvelocity_yawrate)) {
        ctrl_vel_yawrate_pub_.publish(ctrl_ENUvelocity_yawrate);
      }
      break;
    }
    default:
      break;
  }
}

bool PositionController::updatePositionControlReference(
    sensor_msgs::Joy* ctrl_ENUposition_yaw) const {
  ROS_ASSERT(ctrl_ENUposition_yaw);
  ctrl_ENUposition_yaw->axes.clear();
  ctrl_ENUposition_yaw->header.stamp = ros::Time::now();
  ctrl_ENUposition_yaw->header.frame_id = "ground_ENU";

  if (states_.goal_type == GoalType::kNone || !states_.has_ctrl_authority)
    return false;
  // DJI coordinate transform.
  double x_off = states_.goal.position_W.x() -
                 states_.mav.position_W.x();  // Position offset x.
  double y_off = states_.goal.position_W.y() -
                 states_.mav.position_W.y();  // Position offset y.

  // Control action.
  ctrl_ENUposition_yaw->axes.push_back(x_off);
  ctrl_ENUposition_yaw->axes.push_back(y_off);
  double z_off = 0.0;
  if (params_.altitude_mode == AltitudeMode::kAGL) {
    z_off = states_.mav.position_W.z() - states_.agl;
  }
  ctrl_ENUposition_yaw->axes.push_back(states_.goal.position_W.z() + z_off);
  ctrl_ENUposition_yaw->axes.push_back(states_.goal.position_W.z());
  ctrl_ENUposition_yaw->axes.push_back(states_.goal.getYaw());

  // Publish reference for visualization.
  geometry_msgs::PoseStamped ref;
  mav_msgs::msgPoseStampedFromEigenTrajectoryPoint(states_.goal, &ref);
  ref.header = ctrl_ENUposition_yaw->header;
  pos_yaw_ref_pub_.publish(ref);

  // Publish position error
  Eigen::Vector3d position_error(x_off, y_off, z_off);
  std_msgs::Float32 pos_error_msg;
  pos_error_msg.data = position_error.norm();
  pos_error_pub_.publish(pos_error_msg);

  return true;
}

bool PositionController::updateVelocityControlReference(
    sensor_msgs::Joy* ctrl_ENUvelocity_yawrate) const {
  ROS_ASSERT(ctrl_ENUvelocity_yawrate);
  ctrl_ENUvelocity_yawrate->axes.clear();
  ctrl_ENUvelocity_yawrate->header.stamp = ros::Time::now();
  ctrl_ENUvelocity_yawrate->header.frame_id = "ground_ENU";

  const std::string kWarningPrefix = "Cannot update control reference. ";
  if (!states_.has_position) {
    ROS_WARN_STREAM(kWarningPrefix << "No position.");
    return false;
  }
  if (!states_.has_velocity) {
    ROS_WARN_STREAM(kWarningPrefix << "No linear velocity.");
    return false;
  }
  if (!states_.has_translational_acceleration) {
    ROS_WARN_STREAM(kWarningPrefix << "No linear acceleration.");
    return false;
  }
  if (!states_.has_attitude) {
    ROS_WARN_STREAM(kWarningPrefix << "No attitude.");
    return false;
  }
  if (!states_.has_angular_velocity) {
    ROS_WARN_STREAM(kWarningPrefix << "No angular velocity.");
    return false;
  }
  if (!states_.has_angular_acceleration) {
    ROS_WARN_STREAM(kWarningPrefix << "No angular acceleration.");
    return false;
  }
  if (!states_.has_ctrl_authority) {
    ROS_WARN_STREAM(kWarningPrefix << "No control authority.");
    return false;
  }
  if (params_.altitude_mode == kAGL && !states_.has_agl) {
    ROS_WARN_STREAM(kWarningPrefix << "No AGL.");
    return false;
  }
  if (states_.goal_type == GoalType::kNone) {
    ROS_WARN_STREAM(kWarningPrefix << "Invalid goal type.");
    return false;
  }

  // Translation.
  Eigen::Vector3d position_error =
      states_.goal.position_W - states_.mav.position_W;
  if (params_.altitude_mode == kAGL) {
    position_error.z() = states_.goal.position_W.z() - states_.agl;
  }
  const Eigen::Vector3d velocity_error =
      states_.goal.velocity_W - states_.mav.velocity_W;
  const Eigen::Vector3d acceleration_W =
      states_.mav.orientation_W_B.toRotationMatrix() *
      states_.mav.acceleration_B;
  const Eigen::Vector3d acceleration_error =
      states_.goal.acceleration_W - acceleration_W;
  const Eigen::Vector3d dv =
      Eigen::Vector3d(params_.K_px, params_.K_py, params_.K_pz).asDiagonal() *
          position_error +
      Eigen::Vector3d(params_.K_vx, params_.K_vy, params_.K_vz).asDiagonal() *
          velocity_error +
      Eigen::Vector3d(params_.K_ax, params_.K_ay, params_.K_az).asDiagonal() *
          acceleration_error;

  const Eigen::Vector3d v_ref = states_.mav.velocity_W + dv;
  ctrl_ENUvelocity_yawrate->axes.push_back(v_ref.x());
  ctrl_ENUvelocity_yawrate->axes.push_back(v_ref.y());
  ctrl_ENUvelocity_yawrate->axes.push_back(v_ref.z());

  // Orientation.
  // Map the yaw error to lie in the range [-pi, pi].
  const double yaw_error = angles::shortest_angular_distance(
      mav_msgs::yawFromQuaternion(states_.mav.orientation_W_B),
      states_.goal.getYaw());
  const Eigen::Vector3d angular_velocity_W =
      states_.mav.orientation_W_B.toRotationMatrix() *
      states_.mav.angular_velocity_B;
  const double yaw_rate_error =
      states_.goal.getYawRate() - angular_velocity_W.z();
  const Eigen::Vector3d angular_acceleration_W =
      states_.mav.orientation_W_B.toRotationMatrix() *
      states_.mav.angular_acceleration_B;
  const double yaw_acc_error =
      states_.goal.getYawAcc() - angular_acceleration_W.z();
  const double d_omega = params_.K_yaw * yaw_error +
                         params_.K_yaw_rate * yaw_rate_error +
                         params_.K_yaw_acc * yaw_acc_error;

  const double omega_ref = angular_velocity_W.z() + d_omega;
  ctrl_ENUvelocity_yawrate->axes.push_back(omega_ref);

  limitVelocityControlReference(ctrl_ENUvelocity_yawrate);

  // Publish reference for visualization.
  geometry_msgs::PoseStamped ref;
  mav_msgs::msgPoseStampedFromEigenTrajectoryPoint(states_.goal, &ref);
  ref.header = ctrl_ENUvelocity_yawrate->header;
  pos_yaw_ref_pub_.publish(ref);

  // Publish position error
  std_msgs::Float32 pos_error_msg;
  pos_error_msg.data = position_error.norm();
  pos_error_pub_.publish(pos_error_msg);

  return true;
}

void PositionController::limitVelocityControlReference(
    sensor_msgs::Joy* ctrl_ENUvelocity_yawrate) const {
  ROS_ASSERT(ctrl_ENUvelocity_yawrate);
  ROS_ASSERT(ctrl_ENUvelocity_yawrate->axes.size() != 4);

  // v_xy_max.
  double v_xy = Eigen::Vector2d(ctrl_ENUvelocity_yawrate->axes[0],
                                ctrl_ENUvelocity_yawrate->axes[1])
                    .norm();
  if (v_xy > params_.v_xy_max) {
    Eigen::Vector2d v_xy_new =
        Eigen::Vector2d(ctrl_ENUvelocity_yawrate->axes[0],
                        ctrl_ENUvelocity_yawrate->axes[1])
            .normalized() *
        params_.v_xy_max;
    ctrl_ENUvelocity_yawrate->axes[0] = v_xy_new.x();
    ctrl_ENUvelocity_yawrate->axes[1] = v_xy_new.y();
  }

  // v_z.
  if (ctrl_ENUvelocity_yawrate->axes[2] > params_.v_z_up_max) {
    ctrl_ENUvelocity_yawrate->axes[2] = params_.v_z_up_max;
  } else if (ctrl_ENUvelocity_yawrate->axes[2] < -params_.v_z_down_max) {
    ctrl_ENUvelocity_yawrate->axes[2] = -params_.v_z_down_max;
  }

  // Yaw rate.
  if (ctrl_ENUvelocity_yawrate->axes[3] > params_.yaw_rate_max) {
    ctrl_ENUvelocity_yawrate->axes[3] = params_.yaw_rate_max;
  } else if (ctrl_ENUvelocity_yawrate->axes[3] < -params_.yaw_rate_max) {
    ctrl_ENUvelocity_yawrate->axes[3] = -params_.yaw_rate_max;
  }
}

void PositionController::receiveVelocity(
    const geometry_msgs::Vector3StampedConstPtr& msg) {
  if (states_.goal_type == GoalType::kNone || !states_.has_ctrl_authority)
    return;

  states_.has_velocity = true;
  tf::vectorMsgToEigen(msg->vector, states_.mav.velocity_W);
}

void PositionController::receiveAGL(
    const geometry_msgs::PointStampedConstPtr& msg) {
  ROS_INFO_ONCE("Received first AGL.");
  states_.has_agl = true;
  states_.agl = msg->point.z;
}

void PositionController::receiveAcceleration(
    const geometry_msgs::Vector3StampedConstPtr& msg) {
  if (states_.goal_type == GoalType::kNone || !states_.has_ctrl_authority)
    return;

  states_.has_translational_acceleration = true;
  Eigen::Vector3d acceleration_W;
  tf::vectorMsgToEigen(msg->vector, acceleration_W);
  states_.mav.acceleration_B =
      states_.mav.orientation_W_B.inverse().toRotationMatrix() * acceleration_W;
}

void PositionController::receiveAttitude(
    const geometry_msgs::QuaternionStampedConstPtr& msg) {
  if (states_.goal_type == GoalType::kNone || !states_.has_ctrl_authority)
    return;

  states_.has_attitude = true;
  tf::quaternionMsgToEigen(msg->quaternion, states_.mav.orientation_W_B);
}

void PositionController::receiveAngularVelocity(
    const geometry_msgs::Vector3StampedConstPtr& msg) {
  if (states_.goal_type == GoalType::kNone || !states_.has_ctrl_authority)
    return;

  // Angular velocity.
  states_.has_angular_velocity = true;
  tf::vectorMsgToEigen(msg->vector, states_.mav.angular_velocity_B);

  // Angular acceleration.
  const geometry_msgs::Vector3Stamped previous_angular_velocity_B =
      states_.angular_velocity_B;
  states_.angular_velocity_B = *msg;
  if (previous_angular_velocity_B.header.stamp.isValid()) {
    states_.has_angular_acceleration = true;
    Eigen::Vector3d omega, omega_prev;
    tf::vectorMsgToEigen(states_.angular_velocity_B.vector, omega);
    tf::vectorMsgToEigen(previous_angular_velocity_B.vector, omega_prev);
    const double dt = (states_.angular_velocity_B.header.stamp -
                       previous_angular_velocity_B.header.stamp)
                          .toSec();
    states_.mav.angular_acceleration_B = (omega - omega_prev) / dt;
  }
}

void PositionController::receiveRC(const sensor_msgs::JoyConstPtr& msg) {
  // Invalidate goal on new mode.
  if (msg->axes[4] != states_.flight_mode_axis) {
    ROS_INFO_COND(states_.goal_type != GoalType::kNone,
                  "Invalidate current goal.");
    ROS_INFO_COND(states_.has_ctrl_authority, "Release control authority.");
    std_srvs::Empty::Request req;
    std_srvs::Empty::Response res;
    while (!releaseControlAuthority(req, res)) {
      ros::Duration(0.1).sleep();
    }
  }
  states_.flight_mode_axis = msg->axes[4];
}

}  // namespace fm_control
