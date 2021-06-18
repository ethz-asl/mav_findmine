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

#ifndef FM_CONTROL_ENU_CONVERSION_H
#define FM_CONTROL_ENU_CONVERSION_H

#include <geotf/geodetic_converter.h>
#include <libsbp_ros_msgs/MsgBasePosEcef.h>
#include <sensor_msgs/NavSatFix.h>

namespace fm_control {

// Publish TF between DJI ENU frame and RTK ENU frame.
class EnuConversion {
 public:
  EnuConversion(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);

 private:
  void subscribeTopics();
  ros::Subscriber dji_enu_sub_;
  void receiveDjiEnu(const sensor_msgs::NavSatFixConstPtr& msg);
  ros::Subscriber rtk_enu_sub_;
  void receiveRtkEnu(const libsbp_ros_msgs::MsgBasePosEcefConstPtr& msg);

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  geotf::GeodeticConverter geotf_;
};

}  // namespace fm_control

#endif  // FM_CONTROL_ENU_CONVERSION_H
