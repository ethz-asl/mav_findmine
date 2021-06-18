# MIT License
#
# Copyright (c) 2020 Rik Baehnemann, ASL, ETH Zurich, Switzerland
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

UBUNTU_VERSION=$(lsb_release -cs)

if [ $UBUNTU_VERSION = xenial ] ; then
  ROS_NAME="kinetic"
elif [ $UBUNTU_VERSION = bionic ] ; then
  ROS_NAME="melodic"
fi

# Put CPU into performance mode
sudo systemctl disable ondemand

# --- SET UP ROS ---
# Add ROS repository to package manager.
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Add key to ros repository.
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# Update package manager.
sudo apt update

# Install ROS.
sudo apt install ros-${ROS_NAME}-ros-base -y

# Initialize rosdep.
sudo rosdep init
rosdep update

# Dependencies for building packages.
sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential -y

# mav_comm
sudo apt install libeigen3-dev -y

# geo_tf
sudo apt install -y libgdal-dev
sudo apt install -y ros-${ROS_NAME}-tf-conversions

# rviz satellite
sudo apt install -y libogre-1.9-dev
sudo apt install -y ros-${ROS_NAME}-rviz

# mav_tasks
sudo apt install ros-${ROS_NAME}-eigen-conversions -y

# mav_trajectory_generation
sudo apt install libyaml-cpp-dev -y

# fm_control.
sudo apt install ros-${ROS_NAME}-angles -y

# velodyne
sudo apt install ros-${ROS_NAME}-velodyne -y

# ros_task_manager
sudo apt install ros-${ROS_NAME}-move-base -y

# DJI onboard SDK ROS
sudo apt install ros-${ROS_NAME}-nmea-msgs -y

# flir_camera_driver
sudo apt install -y ros-${ROS_NAME}-camera-info-manager ros-${ROS_NAME}-image-proc \
ros-${ROS_NAME}-roslint ros-${ROS_NAME}-diagnostic-updater \
ros-${ROS_NAME}-image-exposure-msgs ros-${ROS_NAME}-wfov-camera-msgs \
ros-${ROS_NAME}-compressed-image-transport
sudo apt install ros-melodic-rqt-image-view -y

# Unit tests
sudo apt install -y libgoogle-glog-dev libgtest-dev

# fm_mission_planner
sudo apt install qt5-default qttools5-dev-tools libqt5svg5-dev qtmultimedia5-dev -y
sudo apt install -y python-pip python-matplotlib
pip install pymap3d
pip install pandas

# fm_sensor_watchdog
sudo apt install -y ros-${ROS_NAME}-rqt-gui-py

# srv_tools
sudo apt install ros-${ROS_NAME}-stereo-image-proc -y
sudo apt install ros-${ROS_NAME}-camera-calibration-parsers -y

# polygon_coverage_planning
sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 3FA7E0328081BFF6A14DA29AA6A19B38D3D831EF
sudo apt install gnupg ca-certificates apt-transport-https -y
echo "deb https://download.mono-project.com/repo/ubuntu stable-${UBUNTU_VERSION} main" | sudo tee /etc/apt/sources.list.d/mono-official-stable.list
sudo apt update

# mav_state_estimation
sudo add-apt-repository -y ppa:borglab/gtsam-release-4.0

# fm_missions
pip install rospkg

echo
echo "*** SETTING UP ROS ***"
echo

cd ~

# CATKIN dependencies.
sudo apt install -y cmake python-catkin-pkg python-catkin-tools python-empy python-nose python-setuptools build-essential

# Useful aliases.
echo -e '\n# Aliases' >> ~/.bashrc
echo 'alias cb="catkin build"' >> ~/.bashrc
echo 'alias cbt="catkin build --this"' >> ~/.bashrc
echo 'alias cbtn="catkin build --this --no-deps"' >> ~/.bashrc

# create catkin workspace
source /opt/ros/${UBUNTU_VERSION}/setup.bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin init
catkin config --extend /opt/ros/${ROS_NAME}
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin config --merge-devel
catkin config --install
source /opt/ros/${ROS_NAME}/setup.bash
cd src/
wstool init
wstool set -y --git mav_findmine git@github.com:ethz-asl/mav_findmine.git -v v1.0.5
wstool update -j8
wstool merge --merge-keep -y ./mav_findmine/install/dependencies.rosinstall
wstool update -j8
wstool merge --merge-keep -y ./ethz_piksi_ros/piksi_multi_cpp/install/dependencies.rosinstall
wstool merge --merge-keep -y ./polygon_coverage_planning/install/dependencies.rosinstall
wstool merge --merge-keep -y ./mav_trajectory_generation/install/mav_trajectory_generation_ssh.rosinstall
wstool update -j8

# Install libraries.
./ethz_piksi_ros/piksi_multi_cpp/install/prepare-jenkins-slave.sh # Piksi
./mav_findmine/install/install_blackfly.sh # Blackfly
./mav_findmine/install/install_dji_sdk.sh # DJI
./polygon_coverage_planning/install/prepare-jenkins-slave.sh # Polygon coverage planning
./mav_state_estimation/install/prepare-jenkins-slave.sh # MAV state estimation

cd ~/catkin_ws/

catkin build -c
source devel/setup.bash

cd ~
rm -rf mav_findmine
mkdir -p bags
mkdir -p trajectories

# add ros to bash
sh -c 'echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc'
sh -c 'echo "export HOSTNAME" >> ~/.bashrc'

# Add useful startup functions to bash.
cat ~/catkin_ws/src/mav_findmine/install/startup_shortcuts >> ~/.bashrc
