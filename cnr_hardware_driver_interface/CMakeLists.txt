cmake_minimum_required(VERSION 3.1)
project(cnr_hardware_driver_interface)

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/cnrConfigMacros.cmake)

cnr_set_flags()


execute_process(COMMAND uname -r OUTPUT_VARIABLE UNAME_RESULT OUTPUT_STRIP_TRAILING_WHITESPACE)
message(STATUS " Kernel version: " ${UNAME_RESULT})
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
  LIBRARIES cnr_hardware_driver_interface
  CATKIN_DEPENDS configuration_msgs cnr_controller_interface_params cnr_controller_manager_interface
    cnr_hardware_interface cnr_logger realtime_utilities nodelet roscpp
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS} )

## Declare a C++ library
add_library(${PROJECT_NAME} src/${PROJECT_NAME}/cnr_hardware_driver_interface.cpp )
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} )
cnr_target_compile_options(${PROJECT_NAME})

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
  cnr_target_compile_options(${PROJECT_NAME}_test)

  if(ENABLE_COVERAGE_TESTING)
    set(COVERAGE_EXCLUDES "*/test*")
    add_code_coverage(
      NAME coverage_report
      DEPENDENCIES ${PROJECT_NAME}_test
    )
  endif()
endif()
