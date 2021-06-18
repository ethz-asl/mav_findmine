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

#ifndef FM_CONTROL_POSITION_CONTROLLER_H
#define FM_CONTROL_POSITION_CONTROLLER_H

#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <mav_planning_msgs/PolynomialTrajectory4D.h>
#include <mav_trajectory_generation/trajectory.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>

#include "fm_control/PositionControllerConfig.h"

namespace fm_control {

enum GoalType { kPose = 0, kPolynomialTrajectory, kNone };
enum ControlType { kPosition = 0, kVelocity = 1 };
enum AltitudeMode { kENU = 0, kAGL = 1 };

// DJI position control in local ENU frame with fead forward velocity.
// TODO(rikba): Proper state machine, check odometry stamps, odometry messages,
// watch telemetry,...
class PositionController {
 public:
  PositionController(const ros::NodeHandle& nh,
                     const ros::NodeHandle& nh_private);

 private:
  void advertiseServices();
  void subscribeServices();
  void advertiseTopics();
  void subscribeTopics();

  // Obtain and release control authority.
  ros::ServiceServer obtain_ctrl_authority_srv_;
  ros::ServiceServer release_ctrl_authority_srv_;
  bool obtainControlAuthority(std_srvs::Empty::Request& request,
                              std_srvs::Empty::Response& response);
  bool releaseControlAuthority(std_srvs::Empty::Request& request,
                               std_srvs::Empty::Response& response);

  // Service to switch control frame (ENU vs AGL)
  ros::ServiceServer switch_control_frame_srv_;
  bool switchControlFrame(std_srvs::SetBool::Request& request,
                          std_srvs::SetBool::Response& response);

  // Enable and disable control authority.
  ros::ServiceClient sdk_ctrl_authority_client_;

  // Publishes reference position/velocity commands to the DJI SDK.
  ros::Publisher ctrl_pos_yaw_pub_;
  ros::Publisher ctrl_vel_yawrate_pub_;
  // Publishes the absolute position error
  ros::Publisher pos_error_pub_;
  // Publishes control authority status
  ros::Publisher ctrl_auth_pub_;
  // Publishes control reference for visualization.
  ros::Publisher pos_yaw_ref_pub_;
  // Subscribes to the fused ENU local position at 50 Hz. Runs control loop.
  ros::Subscriber local_position_sub_;
  void receiveLocalPosition(const geometry_msgs::PointStampedConstPtr& msg);
  // Subscribes to the ENU velocity at 50 Hz.
  ros::Subscriber velocity_sub_;
  void receiveVelocity(const geometry_msgs::Vector3StampedConstPtr& msg);
  // Subscribes to fused AGL estimate.
  ros::Subscriber agl_sub_;
  void receiveAGL(const geometry_msgs::PointStampedConstPtr& msg);

  // Subscribes to fused ENU acceleration at 100 Hz.
  ros::Subscriber acceleration_sub_;
  void receiveAcceleration(const geometry_msgs::Vector3StampedConstPtr& msg);

  // Subscribes to fused ENU attitude at 100 Hz.
  ros::Subscriber attitude_sub_;
  void receiveAttitude(const geometry_msgs::QuaternionStampedConstPtr& msg);

  // Subscribes to fused ENU angular velocity at 100 Hz.
  ros::Subscriber angular_velocity_sub_;
  void receiveAngularVelocity(const geometry_msgs::Vector3StampedConstPtr& msg);

  // Subscribes to the goal.
  ros::Subscriber goal_sub_;
  void receiveGoal(const geometry_msgs::PoseStampedConstPtr& msg);

  // Subscribes to polynomial trajectories.
  ros::Subscriber poly_trajectory_sub_;
  void receivePolynomialTrajectory(
      const mav_planning_msgs::PolynomialTrajectory4D& msg);

  // Update the goal pose.
  void updateGoalPose();

  // Watch the remote.
  ros::Subscriber rc_sub_;
  void receiveRC(const sensor_msgs::JoyConstPtr& msg);

  // Compute controls.
  bool updatePositionControlReference(
      sensor_msgs::Joy* ctrl_ENUposition_yaw) const;

  bool updateVelocityControlReference(
      sensor_msgs::Joy* ctrl_ENUvelocity_yawrate) const;
  void limitVelocityControlReference(
      sensor_msgs::Joy* ctrl_ENUvelocity_yawrate) const;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  struct States {
    States();
    bool has_ctrl_authority;
    GoalType goal_type;
    bool has_position;
    bool has_velocity;
    bool has_translational_acceleration;
    bool has_attitude;
    bool has_angular_velocity;
    bool has_angular_acceleration;
    bool has_agl;
    int flight_mode_axis;
    geometry_msgs::Vector3Stamped angular_velocity_B;
    mav_msgs::EigenTrajectoryPoint goal;
    mav_trajectory_generation::Trajectory trajectory;
    ros::Time trajectory_start;
    mav_msgs::EigenMavState mav;
    double agl;
    void reset();
  };
  States states_;

  struct Parameters {
    Parameters();
    // Position/Angle error gains.
    double K_px;
    double K_py;
    double K_pz;
    double K_yaw;
    // Velocity/angular rate error gains.
    double K_vx;
    double K_vy;
    double K_vz;
    double K_yaw_rate;
    // Acceleration error gains.
    double K_ax;
    double K_ay;
    double K_az;
    double K_yaw_acc;
    // Limits.
    double v_xy_max;
    double v_z_up_max;
    double v_z_down_max;
    double yaw_rate_max;
    // Control type.
    ControlType control_type;
    // Use altitude above ground level.
    AltitudeMode altitude_mode;

    // Minimum transition altitude AGL to switch from
    //  AGL to ENU in [m]
    double min_transition_altitude;
  };
  Parameters params_;

  dynamic_reconfigure::Server<PositionControllerConfig> dyn_config_server_;
  void dynamicReconfigure(PositionControllerConfig& config, uint32_t level);
  void intializeDynamicReconfigure();
};

}  // namespace fm_control

#endif  // FM_CONTROL_POSITION_CONTROLLER_H
