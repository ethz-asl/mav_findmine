cmake_minimum_required(VERSION 2.8.3)
project(fm_mission_planner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

find_package(catkin_simple REQUIRED)
find_package(Eigen3 REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)
set(CMAKE_BUILD_TYPE Release)

######
# QT #
######
set(CMAKE_AUTOMOC ON)
if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  ## pull in all required include dirs, define QT_LIBRARIES, etc.
  include(${QT_USE_FILE})
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
  set(QT_LIBRARIES Qt5::Widgets)
endif()

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
# Header files as explained in https://answers.ros.org/question/215487/could-not-load-panel-in-rviz-pluginlibfactory-the-plugin-for-class/
set(HEADER_FILES
  include/fm_mission_planner/coverage_widget.h
  include/fm_mission_planner/hotpoint_widget.h
  include/fm_mission_planner/mission_planner_panel.h
  include/fm_mission_planner/rectangle_widget.h
  include/fm_mission_planner/trajectory_widget.h
  include/fm_mission_planner/strip_widget.h
)
set(SRC_FILES
  src/coverage_widget.cc
  src/hotpoint_widget.cc
  src/mission.cc
  src/mission_planner_panel.cc
  src/rectangle_widget.cc
  src/trajectory_widget.cc
  src/strip_widget.cc
)
cs_add_library(${PROJECT_NAME} ${SRC_FILES} ${HEADER_FILES})
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${YamlCpp_LIBRARIES})

catkin_python_setup()
cs_install()
cs_export()
