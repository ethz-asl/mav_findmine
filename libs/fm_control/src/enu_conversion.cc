/*
MIT License

Copyright (c) 2021 Rik Baehnemann, ASL, ETH Zurich, Switzerland

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

#include <ros/ros.h>
#include <Eigen/Dense>

#include "fm_control/enu_conversion.h"

namespace fm_control {
EnuConversion::EnuConversion(const ros::NodeHandle& nh,
                             const ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private) {
  geotf_.initFromRosParam();
  subscribeTopics();
}

void EnuConversion::subscribeTopics() {
  const int kQueueSize = 1;
  dji_enu_sub_ = nh_.subscribe("dji_sdk/local_frame_ref", kQueueSize,
                               &EnuConversion::receiveDjiEnu, this);
  rtk_enu_sub_ = nh_.subscribe("piksi/position_receiver_0/sbp/base_pos_ecef",
                               kQueueSize, &EnuConversion::receiveRtkEnu, this);
}

void EnuConversion::receiveDjiEnu(const sensor_msgs::NavSatFixConstPtr& msg) {
  geotf_.removeFrame("dji_enu");
  geotf_.addFrameByENUOrigin("dji_enu", msg->latitude, msg->longitude,
                             msg->altitude);
}

void EnuConversion::receiveRtkEnu(
    const libsbp_ros_msgs::MsgBasePosEcefConstPtr& msg) {
  Eigen::Affine3d rtk_coords(Eigen::Affine3d::Identity());
  rtk_coords.translation().x() = msg->x;
  rtk_coords.translation().y() = msg->y;
  rtk_coords.translation().z() = msg->z;
  ROS_WARN_COND(
      !geotf_.publishAsTf("ecef", rtk_coords, "enu"),
      "Could not publish DJI ENU to RTK ENU transformation. No DJI ENU?");
}

}  // namespace fm_control
