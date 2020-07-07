project(cnr_topics_hardware_interface)

cmake_minimum_required(VERSION 3.1)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

######

add_compile_options(-funroll-loops -Wall -Ofast)
set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  control_msgs
  cnr_hardware_nodelet_interface
  cnr_controller_interface
  controller_manager
  diagnostic_msgs
  geometry_msgs
  hardware_interface
  subscription_notifier
  nodelet
  roscpp
  sensor_msgs
  cnr_hardware_interface
  name_sorting
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cnr_topics_hardware_interface
  CATKIN_DEPENDS control_msgs cnr_hardware_nodelet_interface cnr_controller_interface controller_manager diagnostic_msgs geometry_msgs hardware_interface  subscription_notifier nodelet roscpp sensor_msgs cnr_hardware_interface  name_sorting
  DEPENDS 
)

include_directories  ( include         ${catkin_INCLUDE_DIRS} )
add_library          ( ${PROJECT_NAME} src/${PROJECT_NAME}/cnr_topics_robot_hw.cpp )
add_dependencies     ( ${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
target_link_libraries( ${PROJECT_NAME} ${catkin_LIBRARIES} )


