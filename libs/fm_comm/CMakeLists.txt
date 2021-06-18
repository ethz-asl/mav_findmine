cmake_minimum_required(VERSION 2.8.3)
project(fm_comm)

add_definitions(-std=c++11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

find_package(catkin_simple REQUIRED)
find_package(Eigen3 REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/dji_interface.cc
)
target_link_libraries(${PROJECT_NAME})

#########
# TESTS #
#########
catkin_add_gtest(test_tf
  test/tf-test.cpp
)
target_link_libraries(test_tf ${PROJECT_NAME})

cs_install()
cs_export()
