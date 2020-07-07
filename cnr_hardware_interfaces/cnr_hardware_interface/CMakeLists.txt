cmake_minimum_required(VERSION 3.1)

project(cnr_hardware_interface)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  cnr_controller_manager_interface
  cnr_controller_interface
  controller_manager
  diagnostic_updater
  diagnostic_msgs
  hardware_interface
  nodelet
  roscpp
  sensor_msgs
  configuration_msgs
  realtime_utilities
  pluginlib
  roslint
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES cnr_hardware_interface
 CATKIN_DEPENDS cnr_controller_manager_interface 
                cnr_controller_interface 
                controller_manager 
                diagnostic_updater 
                diagnostic_msgs 
                hardware_interface
                nodelet roscpp
                sensor_msgs
                configuration_msgs
                realtime_utilities
                pluginlib
 DEPENDS 
)

include_directories(include include/internal ${catkin_INCLUDE_DIRS} )
add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/cnr_robot_hw.cpp
)

add_dependencies(cnr_hardware_interface ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cnr_hardware_interface
  ${catkin_LIBRARIES}
)

set(ROSLINT_CPP_OPTS "--filter=-runtime/references,-runtime/int,-build/header_guard --linelength=150")
roslint_cpp(src/${PROJECT_NAME}/cnr_robot_hw.cpp include/${PROJECT_NAME}/cnr_robot_hw.h)