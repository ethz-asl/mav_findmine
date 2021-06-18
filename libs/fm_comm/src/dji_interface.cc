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

#include "fm_comm/dji_interface.h"

#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <ros/ros.h>
#include <ros/topic.h>
#include <sensor_msgs/NavSatFix.h>

namespace fm_comm {

constexpr double kTimeOutS = 1.0;  // Message timeout.

bool queryStateENU(ros::NodeHandle* nh, mav_msgs::EigenMavState* mav_state) {
  ROS_ASSERT(mav_state);

  // Orientation.
  if (!queryOrientation(nh, &(mav_state->orientation_W_B))) return false;
  if (!queryAngularVelocity(nh, &(mav_state->angular_velocity_B))) return false;
  mav_state->angular_acceleration_B = Eigen::Vector3d::Zero();

  // Translation.
  if (!queryLocalPosition(nh, &(mav_state->position_W))) return false;
  if (!queryVelocity(nh, &(mav_state->velocity_W))) return false;

  Eigen::Vector3d acceleration_W;
  if (!queryAcceleration(nh, &acceleration_W)) return false;

  mav_state->acceleration_B =
      mav_state->orientation_W_B.inverse().toRotationMatrix() * acceleration_W;

  return true;
}

// Receive current local position.
bool queryLocalPosition(ros::NodeHandle* nh, Eigen::Vector3d* position_W) {
  ROS_ASSERT(position_W);
  geometry_msgs::PointStampedConstPtr msg =
      ros::topic::waitForMessage<geometry_msgs::PointStamped>(
          "dji_sdk/local_position", *nh, ros::Duration(kTimeOutS));
  if (msg == nullptr) {
    ROS_ERROR("Failed querying local position.");
    return false;
  }

  tf::pointMsgToEigen(msg->point, *position_W);
  return true;
}

bool queryVelocity(ros::NodeHandle* nh, Eigen::Vector3d* velocity_W) {
  ROS_ASSERT(velocity_W);
  geometry_msgs::Vector3StampedConstPtr msg =
      ros::topic::waitForMessage<geometry_msgs::Vector3Stamped>(
          "dji_sdk/velocity", *nh, ros::Duration(kTimeOutS));
  if (msg == nullptr) {
    ROS_ERROR("Failed querying velocity.");
    return false;
  }

  tf::vectorMsgToEigen(msg->vector, *velocity_W);
  return true;
}

bool queryAcceleration(ros::NodeHandle* nh, Eigen::Vector3d* acceleration_W) {
  ROS_ASSERT(acceleration_W);
  geometry_msgs::Vector3StampedConstPtr msg =
      ros::topic::waitForMessage<geometry_msgs::Vector3Stamped>(
          "dji_sdk/acceleration_ground_fused", *nh, ros::Duration(kTimeOutS));
  if (msg == nullptr) {
    ROS_ERROR("Failed querying acceleration.");
    return false;
  }

  tf::vectorMsgToEigen(msg->vector, *acceleration_W);
  return true;
}

bool queryOrientation(ros::NodeHandle* nh,
                      Eigen::Quaterniond* orientation_W_B) {
  ROS_ASSERT(orientation_W_B);
  geometry_msgs::QuaternionStampedConstPtr msg =
      ros::topic::waitForMessage<geometry_msgs::QuaternionStamped>(
          "dji_sdk/attitude", *nh, ros::Duration(kTimeOutS));
  if (msg == nullptr) {
    ROS_ERROR("Failed querying orientation.");
    return false;
  }

  tf::quaternionMsgToEigen(msg->quaternion, *orientation_W_B);
  return true;
}

bool queryAngularVelocity(ros::NodeHandle* nh,
                          Eigen::Vector3d* angular_velocity_B) {
  ROS_ASSERT(angular_velocity_B);
  geometry_msgs::Vector3StampedConstPtr msg =
      ros::topic::waitForMessage<geometry_msgs::Vector3Stamped>(
          "dji_sdk/angular_velocity_fused", *nh, ros::Duration(kTimeOutS));
  if (msg == nullptr) {
    ROS_ERROR("Failed querying angular velocity.");
    return false;
  }

  tf::vectorMsgToEigen(msg->vector, *angular_velocity_B);
  return true;
}

bool queryRefFrameLatLon(ros::NodeHandle* nh, Eigen::Vector3d* origin) {
  ROS_ASSERT(origin);
  sensor_msgs::NavSatFixConstPtr msg =
      ros::topic::waitForMessage<sensor_msgs::NavSatFix>(
          "dji_sdk/local_frame_ref", *nh, ros::Duration(kTimeOutS));
  if (msg == nullptr) {
    ROS_ERROR("Failed querying local frame reference.");
    return false;
  }

  origin->x() = msg->latitude;
  origin->y() = msg->longitude;
  origin->z() = msg->altitude;

  return true;
}

bool queryGPSLatLonAlt(ros::NodeHandle* nh, Eigen::Vector3d* lat_lon_alt) {
  ROS_ASSERT(lat_lon_alt);
  sensor_msgs::NavSatFixConstPtr msg =
      ros::topic::waitForMessage<sensor_msgs::NavSatFix>(
          "dji_sdk/gps_position", *nh, ros::Duration(kTimeOutS));
  if (msg == nullptr) {
    ROS_ERROR("Failed querying GPS lat lon alt.");
    return false;
  }

  lat_lon_alt->x() = msg->latitude;
  lat_lon_alt->y() = msg->longitude;
  lat_lon_alt->z() = msg->altitude;

  return true;
}

void gpsConvertENU(double gps_t_lat, double gps_t_lon, double gps_r_lat,
                   double gps_r_lon, double* ENU_x, double* ENU_y) {
  const double kEarth = 6378137.0;
  const double kDeg2Rad = M_PI / 180.0;
  double d_lon = gps_t_lon - gps_r_lon;
  double d_lat = gps_t_lat - gps_r_lat;
  *ENU_y = kDeg2Rad * d_lat * kEarth;
  *ENU_x = kDeg2Rad * d_lon * kEarth * std::cos(kDeg2Rad * gps_t_lat);
};

}  // namespace fm_comm
