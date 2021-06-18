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

#ifndef FM_COMM_DJI_INTERFACE_H
#define FM_COMM_DJI_INTERFACE_H

#include <mav_msgs/eigen_mav_msgs.h>
#include <ros/ros.h>
#include <Eigen/Core>

namespace fm_comm {

// Interface for standard DJI services with result, cmd_set, cmd_id and
// ack_data response.
template <class Service>
bool callDjiTask(const typename Service::Request& req,
                 ros::ServiceClient& client);

// Debug info published by callDjiTask in case of failure.
template <class Service>
void printResponseWarning(const typename Service::Response& res);

// Single state query.
bool queryStateENU(ros::NodeHandle* nh, mav_msgs::EigenMavState* mav_state);
bool queryLocalPosition(ros::NodeHandle* nh, Eigen::Vector3d* position_W);
bool queryVelocity(ros::NodeHandle* nh, Eigen::Vector3d* velocity_W);
bool queryAcceleration(ros::NodeHandle* nh, Eigen::Vector3d* acceleration_W);
bool queryOrientation(ros::NodeHandle* nh, Eigen::Quaterniond* orientation_W_B);
bool queryAngularVelocity(ros::NodeHandle* nh,
                          Eigen::Vector3d* angular_velocity_B);
bool queryGPSLatLonAlt(ros::NodeHandle* nh, Eigen::Vector3d* lat_lon_alt);

bool queryRefFrameLatLon(ros::NodeHandle* nh, Eigen::Vector3d* origin);

void gpsConvertENU(double gps_t_lat, double gps_t_lon, double gps_r_lat,
                   double gps_r_lon, double* ENU_x, double* ENU_y);

}  // namespace fm_comm

#include "fm_comm/impl/dji_interface_impl.h"

#endif  // FM_COMM_DJI_INTERFACE_H
