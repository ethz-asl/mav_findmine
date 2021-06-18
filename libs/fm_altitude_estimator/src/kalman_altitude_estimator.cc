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

#include <eigen_conversions/eigen_msg.h>
#include <fm_altitude_estimator/kalman_altitude_estimator.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <iostream>
#include <numeric>

namespace fm_altitude_estimator {

KalmanAltitudeEstimator::KalmanAltitudeEstimator(ros::NodeHandle& nh,
                                                 ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private), lidar_buffer_(3) {
  // subscribe to pose estimate
  attitude_subs_ = nh_.subscribe(
      "attitude", 1, &KalmanAltitudeEstimator::AttitudeCallback, this);

  position_subs_ = nh_.subscribe(
      "position", 1, &KalmanAltitudeEstimator::PositionCallback, this);

  // subscribe to range measurements
  lidar_subs_ = nh_.subscribe("lidar_range", 1,
                              &KalmanAltitudeEstimator::LidarCallback, this);

  radar_subs_ = nh_.subscribe("radar_range", 1,
                              &KalmanAltitudeEstimator::RadarCallback, this);

  tof_subs_ = nh_.subscribe("tof_range", 1,
                            &KalmanAltitudeEstimator::TofNormalCallback, this);

  // Filter state publisher
  filter_output_pub_ =
      nh_private_.advertise<geometry_msgs::PointStamped>("state", 1);

  // Read filter config
  process_stdev_ = nh_private_.param<double>("process_stdev", 0.1);

  lidar_update_.stdev =
      nh_private_.param<double>("lidar/measurement_stdev", 0.5);

  lidar_update_.offset = nh_private_.param<double>("lidar/static_offset", 0.0);

  lidar_update_.cut_off_angle =
      nh_private_.param<double>("lidar/static_rejection_angle", 0.0);

  lidar_update_.stdev_scaling_angle =
      nh_private_.param<double>("lidar/measurement_stdev_scaling_angle", 0.0);

  lidar_update_.stdev_scaling_distance = nh_private_.param<double>(
      "lidar/measurement_stdev_scaling_distance", 0.0);

  lidar_update_.signal_quality_min =
      nh_private_.param<int>("lidar/signal_strength_min", 0);

  lidar_update_.maha_max = nh_private_.param<double>("maha_max", 1000);

  radar_update_.stdev =
      nh_private_.param<double>("radar/measurement_stdev", 0.5);

  radar_update_.offset = nh_private_.param<double>("radar/static_offset", 0.0);

  radar_update_.cut_off_angle =
      nh_private_.param<double>("radar/static_rejection_angle", 0.0);

  radar_update_.stdev_scaling_angle =
      nh_private_.param<double>("radar/measurement_stdev_scaling_angle", 0.0);

  radar_update_.stdev_scaling_distance = nh_private_.param<double>(
      "radar/measurement_stdev_scaling_distance", 0.0);

  radar_update_.signal_quality_min = nh_private_.param<int>("radar/snr_min", 0);

  tof_update_.stdev = nh_private_.param<double>("tof/measurement_stdev", 0.5);

  tof_update_.offset = nh_private_.param<double>("tof/static_offset", 0.0);

  tof_update_.cut_off_angle =
      nh_private_.param<double>("tof/static_rejection_angle", 0.0);

  tof_update_.stdev_scaling_angle =
      nh_private_.param<double>("tof/measurement_stdev_scaling_angle", 0.0);

  tof_update_.stdev_scaling_distance =
      nh_private_.param<double>("tof/measurement_stdev_scaling_distance", 0.0);

  tof_update_.signal_quality_min =
      nh_private_.param<int>("tof/signal_quality", 0);

  // debug option
  output_stdev_as_y_ = nh_private_.param<bool>("output_stdev_as_y", true);

  // debug topics:

  // default init
  altitude_ = nh_private_.param<double>("init_altitude", 0.0);

  altitude_cov_ =
      std::pow(nh_private_.param<double>("init_altitude_stdev", 10.0), 2);
  altitude_cov_max_ = nh_private_.param<double>("max_altitude_stdev_rel", 10.0);

  // Write config for debug purposes
  ROS_INFO_STREAM("Initial Altitude: " << altitude_);
  ROS_INFO_STREAM("Initial Altitude Cov: " << altitude_cov_);
  ROS_INFO_STREAM("Process Update Stdev: " << process_stdev_);
  ROS_INFO_STREAM(
      "Sensor config:\n"
      << "Radar: \n"
      << "\t Offset:" << radar_update_.offset << "\n"
      << "\t Stdev:" << radar_update_.stdev << "\n"
      << "\t Stdev Scaling Angle:" << radar_update_.stdev_scaling_angle << "\n"
      << "\t Stdev Scaling Distance:" << radar_update_.stdev_scaling_distance
      << "\n"
      << "\t SNR min:" << unsigned(radar_update_.signal_quality_min) << "\n"
      << "Lidar: \n"
      << "\t Offset:" << lidar_update_.offset << "\n"
      << "\t Stdev:" << lidar_update_.stdev << "\n"
      << "\t Stdev Scaling Angle:" << lidar_update_.stdev_scaling_angle << "\n"
      << "\t Stdev Scaling Distance:" << lidar_update_.stdev_scaling_distance
      << "\n"
      << "\t Signal strength min:" << unsigned(lidar_update_.signal_quality_min)
      << "\n"
      << "TPF: \n"
      << "\t Offset:" << tof_update_.offset << "\n"
      << "\t Stdev:" << tof_update_.stdev << "\n"
      << "\t Stdev Scaling Angle:" << tof_update_.stdev_scaling_angle << "\n"
      << "\t Stdev Scaling Distance:" << tof_update_.stdev_scaling_distance
      << "\n");
}

void KalmanAltitudeEstimator::PublishState() {
  geometry_msgs::PointStamped msg;
  msg.header.frame_id = "AGL";
  msg.header.stamp = last_timestamp_;
  msg.point.z = altitude_;

  if (output_stdev_as_y_) {  // debug messages!
    msg.point.y = std::sqrt(altitude_cov_);
    msg.point.x = std::min(1500.0, maha_);
    // ROS_WARN_THROTTLE(1, "DEBUG: std_dev %.2f", msg.point.y);
  }
  if (std::sqrt(altitude_cov_) > std::sqrt(altitude_cov_max_) &&
      output_stdev_as_y_) {
    ROS_DEBUG_THROTTLE(1, "Altitude std_dev exceeding: %.2f",
                       std::sqrt(altitude_cov_));
  }

  filter_output_pub_.publish(msg);

  // Publish transform from AGL to ENU frame.
  if (last_z_pose_.is_initialized()) {
    geometry_msgs::TransformStamped tf;
    tf.header = msg.header;
    tf.header.frame_id = "ground_ENU";
    tf.child_frame_id = "AGL";
    tf.transform.translation.z = *last_z_pose_ - altitude_;
    tf.transform.rotation.w = 1.0;
    tf_br_.sendTransform(tf);
  }
}

void KalmanAltitudeEstimator::TofNormalCallback(
    const geometry_msgs::PoseStampedConstPtr& msg) {
  // Trigger measurement update
  last_timestamp_ = msg->header.stamp;
  Eigen::Vector3d range_sensor;
  range_sensor << 0.0, 0.0, msg->pose.position.z + tof_update_.offset;
  ROS_INFO_STREAM("tof update");
  MeasurementUpdate(range_sensor, tof_update_);
}

void KalmanAltitudeEstimator::ProcessUpdate(const double delta_z) {
  // Here, we model delta_z as the B*u_k term in
  // x_k+ = A*x_k + B * u_k
  altitude_ = altitude_ + delta_z;

  // Similarly, P is Unity, but Q is our increase of covariance
  // p_k+ = APA^{T} + Q
  altitude_cov_ += std::pow(process_stdev_, 2);

  PublishState();
}

void KalmanAltitudeEstimator::MeasurementUpdate(
    const Eigen::Vector3d& range_sensor,
    const RangeMeasurementUpdateConfig& config) {
  if (!last_attitude_.is_initialized()) {
    ROS_WARN_STREAM_THROTTLE(
        10, "Measurement update failed due to missing attitude.");
    return;
  }

  // first, we get the z-axis aligned component of raw-range.
  Eigen::Matrix3d R_world_sensor = last_attitude_->toRotationMatrix();
  Eigen::Vector3d range_world = R_world_sensor * range_sensor;
  double z_world = range_world.z();

  // scale stdev by total angle (stdev_scaling_angle)
  // and by range (stdev_scaling_distance)
  double scaled_stdev = (1.0 + config.stdev_scaling_angle * (*last_angle_) +
                         config.stdev_scaling_distance * range_sensor.z()) *
                        config.stdev;

  // calculate scalar Kalman gain
  // K = P*H^T * ( H*P*H^T + R)^-1
  // here, P = altitude_cov, H = 1, R = scaled_stdev^2
  double K =
      altitude_cov_ * (1.0 / (altitude_cov_ + std::pow(scaled_stdev, 2)));

  // update states
  // x_k = x_k + K *(z-H*x_k) (note, H = 1 here)
  altitude_ = altitude_ + K * (z_world - altitude_);

  // P_k = (I-K*H)*P_k (note, H = 1, I = 1)
  altitude_cov_ = (1 - K) * altitude_cov_;

  PublishState();
}

void KalmanAltitudeEstimator::LidarCallback(
    const versavis::LidarLiteConstPtr& msg) {
  // Trigger measurement update
  last_timestamp_ = msg->range.header.stamp;

  if (!SanityCheck(msg->range.range, msg->range.min_range, msg->range.max_range,
                   msg->signal_strength, lidar_update_, "lidar")) {
    return;
  }

  lidar_buffer_.push_back(msg->range.range);

  // run median filter
  std::vector<double> lidar_data_;

  if (lidar_buffer_.size() == 3) {
    for (double element : lidar_buffer_) {
      lidar_data_.push_back(element);
    }
  } else {
    return;
  }

  // median filtering
  /*
  std::sort(lidar_data_.begin(), lidar_data_.end());
  double value = lidar_data_[1];
  */
  // mean filtering
  double value =
      std::accumulate(lidar_data_.begin(), lidar_data_.end(), 0.0) / 3.0;

  // check mahalonobis distance
  maha_ = std::sqrt(std::pow(value - altitude_, 2) / altitude_cov_);

  if (maha_ > lidar_update_.maha_max) {
    ROS_WARN_STREAM("Ignoring lidar measurement, maha = "
                    << maha_ << " > " << lidar_update_.maha_max);
    return;
  }

  RangeMeasurementUpdateConfig lidar_changed(lidar_update_);

  lidar_changed.stdev =
      std::max(lidar_update_.stdev,
               lidar_update_.stdev * (1 - std::log10(msg->range.range / 0.75)));

  if (lidar_changed.stdev <= 0) {
    ROS_WARN_STREAM("Invalid Lidar stdev. STDEV= "
                    << lidar_changed.stdev << " alt=" << msg->range.range);
    return;
  }

  Eigen::Vector3d range_sensor;
  range_sensor << 0.0, 0.0, value + lidar_update_.offset;
  ROS_WARN_STREAM_ONCE(
      "First lidar update with stdev = " << lidar_changed.stdev);
  MeasurementUpdate(range_sensor, lidar_changed);
}

void KalmanAltitudeEstimator::RadarCallback(const versavis::UsD1ConstPtr& msg) {
  // Trigger measurement update
  last_timestamp_ = msg->range.header.stamp;

  if (!SanityCheck(msg->range.range, msg->range.min_range, msg->range.max_range,
                   msg->snr, radar_update_, "radar")) {
    return;
  }

  Eigen::Vector3d range_sensor;
  range_sensor << 0.0, 0.0, msg->range.range + radar_update_.offset;
  ROS_WARN_STREAM_ONCE(
      "First radar update with stdev = " << radar_update_.stdev);
  MeasurementUpdate(range_sensor, radar_update_);
}

void KalmanAltitudeEstimator::PositionCallback(
    const geometry_msgs::PointStampedConstPtr& msg) {
  if (!last_z_pose_.is_initialized()) {
    last_z_pose_ = msg->point.z;
    last_timestamp_ = msg->header.stamp;
    // init sensor with measured altitude.
    altitude_ = *last_z_pose_;
    ROS_INFO("Position initialized");
    return;
  }

  // Trigger process update with delta_z.
  double delta_z = msg->point.z - *last_z_pose_;
  last_timestamp_ = msg->header.stamp;
  ProcessUpdate(delta_z);
  last_z_pose_ = msg->point.z;
}

void KalmanAltitudeEstimator::AttitudeCallback(
    const geometry_msgs::QuaternionStampedConstPtr& msg) {
  if (!last_attitude_.is_initialized()) {
    Eigen::Quaterniond quat_init;
    tf::quaternionMsgToEigen(msg->quaternion, quat_init);

    // Explicit first assignement, such that optional<> knows its initialized.
    last_attitude_ = quat_init;
    last_timestamp_ = msg->header.stamp;
    ROS_INFO("Attitude initialized");
    return;
  }

  // Store for Measurement updates
  last_timestamp_ = msg->header.stamp;
  tf::quaternionMsgToEigen(msg->quaternion, *last_attitude_);

  // get absolute angle between -gravity vector in world frame
  // and current body Z- axis
  // (this makes yaw not count for the total angular difference)
  Eigen::Matrix3d R_world_sensor = last_attitude_->toRotationMatrix();
  Eigen::Vector3d rotated = R_world_sensor * Eigen::Vector3d::UnitZ();
  last_angle_ = std::acos(rotated.dot(Eigen::Vector3d::UnitZ()));
}

bool KalmanAltitudeEstimator::SanityCheck(
    const double range, const double range_min, const double range_max,
    const uint8_t signal_quality,
    const RangeMeasurementUpdateConfig& update_config,
    std::string sensor_name) {
  // security checks
  if (signal_quality < update_config.signal_quality_min) {
    ROS_DEBUG_THROTTLE(5.0, "%s ignored, quality = %u", sensor_name.c_str(),
                       signal_quality);
    return false;
  }
  if (range < range_min || range > range_max) {
    std::string error_msg = " ignored, out of range: %.2f";

    ROS_DEBUG_THROTTLE(5.0, "%s ignored, out of range: %.2f",
                       sensor_name.c_str(), range);
    return false;
  }
  if (!last_angle_.is_initialized()) {
    ROS_DEBUG_THROTTLE(5.0, "%s ignored, no attitude information.",
                       sensor_name.c_str());
    return false;
  }
  if ((*last_angle_) > update_config.cut_off_angle) {
    ROS_DEBUG_STREAM(sensor_name
                     << " ignored, high tilt angle: " << (*last_angle_) << " > "
                     << update_config.cut_off_angle);
    return false;
  }
  return true;
}

}  // namespace fm_altitude_estimator
