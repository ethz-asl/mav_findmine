cmake_minimum_required(VERSION 2.8.3)
project(fm_trajectories)

add_definitions(-std=c++11)

find_package(catkin_simple REQUIRED)
find_package(Eigen3 REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)
set(CMAKE_BUILD_TYPE Release)

############
# YAML-CPP #
############
# Link against system catkin yaml-cpp if installed.
find_package(PkgConfig)
find_package(yaml_cpp_catkin QUIET)
if(${yaml_cpp_catkin_FOUND})
    message(STATUS "Found yaml_cpp_catkin, using instead of system library.")
    set(YamlCpp_LIBRARIES ${yaml_cpp_catkin_LIBRARIES})
    set(YamlCpp_INCLUDE_DIRS ${yaml_cpp_catkin_INCLUDE_DIRS})
else()
    message(STATUS "No yaml_cpp_catkin, using yaml-cpp system library instead.")
    pkg_check_modules(YamlCpp REQUIRED yaml-cpp>=0.5)
endif()

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/base_trajectory.cc
  src/circle_trajectory.cc
  src/polygon_trajectory.cc
  src/rectangle_trajectory.cc
  src/strip_trajectory.cc
  src/trajectory_generator.cc
  src/waypoint_trajectory.cc
)
target_link_libraries(${PROJECT_NAME} ${YamlCpp_LIBRARIES})


############
# BINARIES #
############
cs_add_executable(trajectory_generator src/trajectory_generator_node.cc)
target_link_libraries(trajectory_generator ${PROJECT_NAME})

#########
# TESTS #
#########
catkin_add_gtest(test_circle
  test/circle-test.cpp
)
target_link_libraries(test_circle ${PROJECT_NAME})

cs_install()
cs_export()
