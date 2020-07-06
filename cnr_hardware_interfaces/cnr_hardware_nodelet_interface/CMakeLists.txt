cmake_minimum_required(VERSION 3.0.2)
project(cnr_hardware_nodelet_interface)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cnr_controller_interface
  cnr_controller_manager_interface
  cnr_hardware_interface
  cnr_logger
  nodelet
  roscpp
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cnr_hardware_nodelet_interface
  CATKIN_DEPENDS cnr_controller_interface cnr_controller_manager_interface cnr_hardware_interface cnr_logger nodelet roscpp
#  DEPENDS system_lib
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
   src/${PROJECT_NAME}/cnr_hardware_nodelet_interface.cpp
   src/${PROJECT_NAME}/cnr_robot_hw_nodelet.cpp
)

