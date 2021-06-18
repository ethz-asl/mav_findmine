cmake_minimum_required(VERSION 2.8.3)
project(fm_task_manager)

add_definitions(-std=c++17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

################
# DEPENDENCIES #
################
find_package(catkin_simple REQUIRED)
find_package(Eigen3 REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

find_package(DJIOSDK REQUIRED)

############
# BINARIES #
############
cs_add_executable(task_server src/task_server_node.cc src/task_env.cc src/task_server.cc)
target_link_libraries(task_server ${DJIOSDK_LIBRARIES} dl)

#########
# TASKS #
#########
# Generate task plugins in a well defined place.
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION}/tasks)

# Adding tasks as shared library.
ADD_LIBRARY(task_camera SHARED src/tasks/camera.cc)
TARGET_LINK_LIBRARIES(task_camera ${catkin_LIBRARIES} dl)
ADD_DEPENDENCIES(task_camera ${${PROJECT_NAME}_EXPORTED_TARGETS})

ADD_LIBRARY(task_execute_single_trajectory SHARED src/tasks/execute_single_trajectory.cc)
TARGET_LINK_LIBRARIES(task_execute_single_trajectory ${catkin_LIBRARIES} dl)
ADD_DEPENDENCIES(task_execute_single_trajectory ${${PROJECT_NAME}_EXPORTED_TARGETS})

ADD_LIBRARY(task_generate_single_trajectory SHARED src/tasks/generate_single_trajectory.cc)
TARGET_LINK_LIBRARIES(task_generate_single_trajectory ${catkin_LIBRARIES} dl)
ADD_DEPENDENCIES(task_generate_single_trajectory ${${PROJECT_NAME}_EXPORTED_TARGETS})

ADD_LIBRARY(task_landing SHARED src/tasks/landing.cc)
TARGET_LINK_LIBRARIES(task_landing ${catkin_LIBRARIES} dl)
ADD_DEPENDENCIES(task_landing ${${PROJECT_NAME}_EXPORTED_TARGETS})

ADD_LIBRARY(task_logging SHARED src/tasks/logging.cc)
TARGET_LINK_LIBRARIES(task_logging ${catkin_LIBRARIES} dl)
ADD_DEPENDENCIES(task_logging ${${PROJECT_NAME}_EXPORTED_TARGETS})

ADD_LIBRARY(task_postprocessing SHARED src/tasks/postprocessing.cc)
TARGET_LINK_LIBRARIES(task_postprocessing ${catkin_LIBRARIES} dl)
ADD_DEPENDENCIES(task_postprocessing ${${PROJECT_NAME}_EXPORTED_TARGETS})

ADD_LIBRARY(task_take_off SHARED src/tasks/take_off.cc)
TARGET_LINK_LIBRARIES(task_take_off ${catkin_LIBRARIES} dl)
ADD_DEPENDENCIES(task_take_off ${${PROJECT_NAME}_EXPORTED_TARGETS})

ADD_LIBRARY(task_waypoint SHARED src/tasks/waypoint.cc)
TARGET_LINK_LIBRARIES(task_waypoint ${catkin_LIBRARIES} dl)
ADD_DEPENDENCIES(task_waypoint ${${PROJECT_NAME}_EXPORTED_TARGETS})

cs_install()
cs_export()
