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

#include <fm_altitude_estimator/kalman_altitude_estimator.h>
#include <ros/ros.h>

// Standard C++ entry point
int main(int argc, char** argv) {
  // Announce this program to the ROS master
  ros::init(argc, argv, "altitude_estimator_node");
  // Creating the node handles
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  // Creating the relay with ros interface
  fm_altitude_estimator::KalmanAltitudeEstimator estimator(nh, nh_private);

  ros::spin();

  return 0;
}
