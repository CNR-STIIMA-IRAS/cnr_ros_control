cmake_minimum_required(VERSION 3.1)
project(cnr_hardware_nodelet_interface)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

execute_process(COMMAND uname -r OUTPUT_VARIABLE UNAME_RESULT OUTPUT_STRIP_TRAILING_WHITESPACE)
message(-- " Kernel version: " ${UNAME_RESULT})
if(UNAME_RESULT MATCHES "^([^rt]*)rt")
  set(PREEMPTIVE_RT ON CACHE BOOL "Compile library for preemptive machines")
else()
  set(PREEMPTIVE_RT OFF CACHE BOOL "Compile library for preemptive machines")
endif()

find_package(catkin REQUIRED COMPONENTS
  configuration_msgs
  cnr_controller_interface_params
  cnr_controller_manager_interface
  cnr_hardware_interface
  cnr_logger
  realtime_utilities
  nodelet
  roscpp
)

if(CATKIN_ENABLE_TESTING AND ENABLE_COVERAGE_TESTING)
  find_package(code_coverage REQUIRED)
  # Add compiler flags for coverage instrumentation before defining any targets
  APPEND_COVERAGE_COMPILER_FLAGS()
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cnr_hardware_nodelet_interface
  CATKIN_DEPENDS configuration_msgs cnr_controller_interface_params cnr_controller_manager_interface
    cnr_hardware_interface cnr_logger realtime_utilities nodelet roscpp
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS} )

## Declare a C++ library
add_library(${PROJECT_NAME} src/${PROJECT_NAME}/cnr_robot_hw_nodelet.cpp )
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} )
target_compile_options(${PROJECT_NAME} PUBLIC -Wall -faligned-new
        $<$<CONFIG:Release>:-Ofast -funroll-loops -ffast-math >)
if(PREEMPTIVE_RT)
  target_compile_definitions(${PROJECT_NAME} PUBLIC -DPREEMPTIVE_RT=1)
else()
  target_compile_definitions(${PROJECT_NAME} PUBLIC -DPREEMPTIVE_RT=0)
endif()
if(${CMAKE_VERSION} VERSION_GREATER "3.16.0")
target_precompile_headers(${PROJECT_NAME} PUBLIC  <cstring>
                                                  <sstream>
                                                  <thread>
                                                  <memory>
                                                  <map>
                                                  <string>
                                                  <boost/date_time/posix_time/posix_time.hpp>
                                                  <ros/ros.h>
                                                  <nodelet/nodelet.h>
                                                  <nodelet/NodeletLoad.h>
                                                  <nodelet/NodeletUnload.h>
                                                  <nodelet/NodeletList.h>
                                                  <pluginlib/class_loader.h>
                                                  <pluginlib/class_list_macros.h>
                                                  <diagnostic_updater/diagnostic_updater.h>
                                                  <diagnostic_msgs/DiagnosticArray.h>)
endif()

#############
## Install ##
#############

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".git" EXCLUDE
 )

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME})


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
if(CATKIN_ENABLE_TESTING)
  find_package(roscpp REQUIRED)
  find_package(rostest REQUIRED)
  add_rostest_gtest(${PROJECT_NAME}_test test/test.launch test/test.cpp)
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME} ${catkin_LIBRARIES} ${roscpp_LIBRARIES} )

  if(ENABLE_COVERAGE_TESTING)
    set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/test*")
    add_code_coverage(
      NAME ${PROJECT_NAME}_coverage_report
      DEPENDENCIES ${PROJECT_NAME}_test
    )
  endif()
endif()
