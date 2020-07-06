cmake_minimum_required(VERSION 3.1)
project(cnr_controller_manager_interface)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  cnr_controller_interface
  controller_manager
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cnr_controller_manager_interface
  CATKIN_DEPENDS cnr_controller_interface controller_manager
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
   src/${PROJECT_NAME}/cnr_controller_manager_interface.cpp
)


