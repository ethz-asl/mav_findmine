cmake_minimum_required(VERSION 2.8.3)
project(fm_vicon)

add_definitions(-std=c++11)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

############
# BINARIES #
############
catkin_install_python(PROGRAMS scripts/att_relay
                               scripts/compute_tf
                               scripts/gt_relay
                               scripts/pos_relay
		      DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_python_setup()
cs_install()
cs_export()
