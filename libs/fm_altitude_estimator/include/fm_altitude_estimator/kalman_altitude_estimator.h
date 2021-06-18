/*
MIT License

Copyright (c) 2020 Michael Pantic, Rik Baehnemann, ASL, ETH Zurich, Switzerland

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

//
// Created by mpantic on 08.04.19.
//

#ifndef FM_ALTITUDE_ESTIMATOR_KALMAN_ALTITUDE_ESTIMATOR_H
#define FM_ALTITUDE_ESTIMATOR_KALMAN_ALTITUDE_ESTIMATOR_H
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/Range.h>
#include <tf2_ros/transform_broadcaster.h>
#include <versavis/LidarLite.h>
#include <versavis/UsD1.h>
#include <Eigen/Dense>
#include <boost/circular_buffer.hpp>
#include <boost/optional.hpp>

namespace fm_altitude_estimator {

class KalmanAltitudeEstimator {
  struct RangeMeasurementUpdateConfig {
    double stdev;          // Standard deviation in [m]
    double offset;         // static Offset in [m]
    double cut_off_angle;  // static offset for rejecting measurements [rad]
    double
        stdev_scaling_angle;  // Scaling of measurement cov w.r.t attiude [0-x]
    double stdev_scaling_distance;   // altituded dependand stdev
    uint8_t signal_quality_min = 0;  // minimal required signal strength [0-255]
    double maha_max;
  };

 public:
  KalmanAltitudeEstimator(ros::NodeHandle& nh, ros::NodeHandle& nh_private);

 private:
  void LidarCallback(const versavis::LidarLiteConstPtr& msg);
  void RadarCallback(const versavis::UsD1ConstPtr& msg);
  void TofNormalCallback(const geometry_msgs::PoseStampedConstPtr& msg);
  void AttitudeCallback(const geometry_msgs::QuaternionStampedConstPtr& msg);
  void PositionCallback(const geometry_msgs::PointStampedConstPtr& msg);

  void ProcessUpdate(double delta_z);
  void MeasurementUpdate(const Eigen::Vector3d&,
                         const RangeMeasurementUpdateConfig& config);
  bool SanityCheck(const double range, const double range_min,
                   const double range_max, const uint8_t singal_quality,
                   const RangeMeasurementUpdateConfig& update_config,
                   const std::string sensor_name);

  void PublishState();

  // Ros stuff
  ros::NodeHandle& nh_;
  ros::NodeHandle& nh_private_;

  ros::Subscriber lidar_subs_;
  ros::Subscriber radar_subs_;
  ros::Subscriber position_subs_;
  ros::Subscriber attitude_subs_;
  ros::Subscriber tof_subs_;
  ros::Publisher filter_output_pub_;
  ros::Publisher position_agl_pub_;
  ros::Time last_timestamp_;
  bool output_stdev_as_y_;

  tf2_ros::TransformBroadcaster tf_br_;

  boost::circular_buffer<double> lidar_buffer_;

  // Cached measurements
  boost::optional<Eigen::Quaterniond> last_attitude_;
  boost::optional<double> last_z_pose_;
  boost::optional<double> last_angle_;

  // Filter variables
  double altitude_;
  double altitude_cov_;
  double altitude_cov_max_;
  double maha_;

  RangeMeasurementUpdateConfig radar_update_;
  RangeMeasurementUpdateConfig lidar_update_;
  RangeMeasurementUpdateConfig tof_update_;
  double process_stdev_;  // Process update standard deviation in [m]
};
}  // namespace fm_altitude_estimator

#endif  // FM_ALTITUDE_ESTIMATOR_KALMAN_ALTITUDE_ESTIMATOR_H
