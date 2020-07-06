cmake_minimum_required(VERSION 3.1)

project(cnr_fake_hardware_interface)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  cnr_hardware_nodelet_interface
  cnr_hardware_interface
  nodelet
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cnr_fake_hardware_interface
  CATKIN_DEPENDS cnr_hardware_nodelet_interface cnr_hardware_interface  nodelet roscpp
  DEPENDS 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/cnr_fake_robot_hw.cpp
  src/${PROJECT_NAME}/cnr_fake_robot_hw_nodelet.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
