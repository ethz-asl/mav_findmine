cmake_minimum_required(VERSION 2.8.3)
project(fm_control)

add_definitions(-std=c++11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package(DJIOSDK REQUIRED)

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/enu_conversion.cc
  src/pid_controller.cc
  src/position_controller.cc
)
target_link_libraries(${PROJECT_NAME} ${DJIOSDK_LIBRARIES})


############
# BINARIES #
############
cs_add_executable(position_controller src/position_controller_node.cc)
target_link_libraries(position_controller ${PROJECT_NAME})

cs_add_executable(enu_conversion src/enu_conversion_node.cc)
target_link_libraries(enu_conversion ${PROJECT_NAME})

add_dependencies(position_controller ${${PROJECT_NAME}_EXPORTED_TARGETS})

cs_install()
cs_export()
