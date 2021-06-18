# mav_findmine
This repository implements a UAV system for automatic radar and camera surveys.

![system demonstration video](https://user-images.githubusercontent.com/11293852/105842398-65c54800-5fd6-11eb-9e41-edd821c9dad9.gif)

Please cite our accompanying publication when using this repository.
```
Bähnemann, Rik, et al.
"Under the Sand: Navigation and Localization of a Small Unmanned Aerial Vehicle for Landmine Detection with Ground Penetrating Synthetic Aperture Radar"
Field Robotics. 2021 (submitted).
```

## Installation
The system uses [ROS](https://www.ros.org/) as middle ware to implement interfaces to various sensors and the [DJI Onboard SDK](https://developer.dji.com/onboard-sdk/).
Please follow the [supervised installation guide](install#installation-from-source) to install the versioned stack on the onboard computer as well as base station computer and RTK base station.

## Update
The custom dependencies are managed through [wstool](http://wiki.ros.org/wstool).
If not stated otherwise, update the following way.
```
cd ~/catkin_ws/src
catkin clean -y
wstool update -j8
cd mav_findmine
git checkout v1.0.5
cd ..
wstool merge --merge-replace -y ./mav_findmine/install/dependencies.rosinstall
wstool update -j8
catkin build
```

## User manual
The user workflow to perform a survey with either camera or radar is organized in the following steps:
1. [Plan a mission offline](libs/fm_mission_planner)
2. Setup the field test sequentially<br>
2.1 Survey the base station position ([old](https://github.com/ethz-asl/mav_findmine/wiki/05-RTK-Base-Setup#geodetic-survey)/[new](https://github.com/ethz-asl/ethz_piksi_ros/tree/master/piksi_multi_interface#usage))<br>
2.2 Power the UAV
3. Upload the mission file into the `/home/$USER/trajectories` folder on the UAV
4. [Configure the mission](integration/fm_missions#mission-configuration) on the UAV<br>
```configure_mission $UAV_NAME```
5. Open the mission control center<br>
```mission_control $UAV_NAME```
6. Load the trajectory on the UAV<br>
```trajectory_start $UAV_NAME my_trajectory.yaml```
7. Inspect the trajectory in the mission control center and press<br>
<kbd>approve trajectory</kbd>
8. The resulting sensor logs are placed in a folder `/home/$USER/bags/YYMMDDHHMMSS`

## Structure
The repository is separated into two main categories.

### integration
The packages in the [integration folder](integration) gather different external dependencies into single system applications.
| Package                                | Summary                                                                                |
| -------------------------------------- | -------------------------------------------------------------------------------------- |
| [fm_missions](integration/fm_missions) | Scripts to start all system components and state machine to execute automatic survey.  |
| [fm_vicon](integration/fm_vicon)       | Scripts to perform Vicon ground truth evaluation experiments.                          |

### libs
The packages in the [libs folder](libs) are custom self-contained applications tailored for the automatic survey system.
| Package                                             | Summary                                                                                                                 |
| --------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------- |
| [fm_altitude_estimator](libs/fm_altitude_estimator) | A Kalman filter to fuse GNSS change in altitude, lidar, and radar altimeter into single altitude above ground estimate. |
| [fm_comm](libs/fm_comm)                             | Custom message definitions, services, and interface wrappers.                                                           |
| [fm_control](libs/fm_control)                       | A PID controller for DJI M600 Pro.                                                                                      |
| [fm_mission_planner](libs/fm_mission_planner)       | A RVIZ plugin to plan and visualize survey trajectories.                                                                |
| [fm_pix4d](libs/fm_pix4d)                           | A pipeline to prepare rosbags for [Pix4d mapper](https://www.pix4d.com/product/pix4dmapper-photogrammetry-software), create google tiles from Pix4d products, and host the tiles to be used as backdrop in mission planner. |
| [fm_ppk](libs/fm_ppk)                               | Scripts to perform a PPK survey using rtklib and swiss topo correction services.                                        |
| [fm_radar_driver](libs/fm_radar_driver)             | A slightly modified version of the THU GPR driver.                                                                      |
| [fm_sensor_watchdog](libs/fm_sensor_watchdog)       | A PyQT GUI to monitor sensor status and engage a mission.                                                               |
| [fm_task_manager](libs/fm_task_manager)             | A collection of robot tasks, e.g., take off, waypoints, and sensor recording, based on [ros_task_manager](https://github.com/cedricpradalier/ros_task_manager). |
| [fm_trajectories](libs/fm_trajectories)             | A library to create custom trajectories, e.g, circles, strips, and rectangles based on [mav_trajectory_generation](https://github.com/ethz-asl/mav_trajectory_generation). |
