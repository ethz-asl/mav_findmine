cmake_minimum_required(VERSION 2.8.3)
project(fm_missions)

add_definitions(-std=c++11)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

############
# BINARIES #
############
catkin_install_python(PROGRAMS nodes/single_trajectory_mission.py
                      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_python_setup()
cs_install()
cs_export()
