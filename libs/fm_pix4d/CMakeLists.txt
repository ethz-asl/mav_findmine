cmake_minimum_required(VERSION 3.0.2)
project(fm_pix4d)

add_definitions(-std=c++11)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

catkin_python_setup()
cs_install()
cs_export()
