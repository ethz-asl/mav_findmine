cmake_minimum_required(VERSION 2.8.3)
project(fm_altitude_estimator)

add_definitions(-std=c++11)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)
catkin_python_setup()

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
        src/kalman_altitude_estimator.cc
        )
target_link_libraries(${PROJECT_NAME})

############
# BINARIES #
############
cs_add_executable(fm_altitude_estimator_node src/altitude_estimator_node.cc)
target_link_libraries(fm_altitude_estimator_node ${PROJECT_NAME})

catkin_install_python(PROGRAMS scripts/plot
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

cs_install()
cs_export()
