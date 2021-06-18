cmake_minimum_required(VERSION 2.8.3)
project(fm_radar_driver)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

catkin_python_setup()

catkin_install_python(PROGRAMS scripts/detect_circles scripts/export_raw_positions
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

cs_install()
cs_export()
