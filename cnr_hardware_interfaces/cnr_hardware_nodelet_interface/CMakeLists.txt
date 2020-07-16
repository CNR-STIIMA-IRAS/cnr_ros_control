cmake_minimum_required(VERSION 3.1)
project(cnr_hardware_nodelet_interface)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  cnr_controller_interface
  cnr_controller_manager_interface
  cnr_hardware_interface
  cnr_logger
  realtime_utilities
  nodelet
  roscpp
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cnr_hardware_nodelet_interface
  CATKIN_DEPENDS cnr_controller_interface cnr_controller_manager_interface cnr_hardware_interface cnr_logger realtime_utilities nodelet roscpp
#  DEPENDS system_lib
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
   src/${PROJECT_NAME}/cnr_robot_hw_nodelet.cpp
)

