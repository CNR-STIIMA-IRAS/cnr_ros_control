cmake_minimum_required(VERSION 3.1)

project(cnr_controller_interface)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  realtime_utilities cnr_controller_interface_params cnr_hardware_interface controller_interface diagnostic_msgs
  cnr_logger urdf rosdyn_core rosdyn_chain_state subscription_notifier sensor_msgs
)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package (Boost COMPONENTS system REQUIRED)

if(CATKIN_ENABLE_TESTING AND ENABLE_COVERAGE_TESTING)
  find_package(code_coverage REQUIRED)
  # Add compiler flags for coverage instrumentation before defining any targets
  APPEND_COVERAGE_COMPILER_FLAGS()
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS urdf realtime_utilities cnr_controller_interface_params cnr_hardware_interface controller_interface
      cnr_logger rosdyn_core rosdyn_chain_state subscription_notifier sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/cnr_controller_interface/cnr_controller_interface.cpp
                            src/cnr_controller_interface/internal/cnr_handles.cpp )
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_SYSTEM_LIBRARY} Eigen3::Eigen)
target_compile_options(${PROJECT_NAME} PUBLIC -Wall -faligned-new
        $<$<CONFIG:Release>:-Ofast -funroll-loops -ffast-math >)


if(${CMAKE_VERSION} VERSION_GREATER  "3.16.0")
	target_precompile_headers(${PROJECT_NAME} PUBLIC
    <map>
    <stdexcept>
    <mutex>
    <thread>
    <Eigen/Core>
    <ros/ros.h>
    <ros/console.h>
    <ros/time.h>
    <std_msgs/Int64.h>
    <sensor_msgs/JointState.h>
    <urdf_model/model.h>
    <urdf_parser/urdf_parser.h>
    <hardware_interface/joint_state_interface.h>
    <hardware_interface/joint_command_interface.h>
  )
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

#############
## Testing ##
#############


## Add gtest based cpp test target and link libraries
if(CATKIN_ENABLE_TESTING)
  find_package(roscpp REQUIRED)
  find_package(rostest REQUIRED)
  find_package(cnr_fake_hardware_interface REQUIRED)
  add_rostest_gtest(${PROJECT_NAME}_test test/test.launch test/test.cpp)
  target_include_directories(${PROJECT_NAME}_test PUBLIC
    include 
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${cnr_fake_hardware_interface_INCLUDE_DIRS})
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME} ${catkin_LIBRARIES} ${roscpp_LIBRARIES} ${cnr_fake_hardware_interface_LIBRARIES})

  if(ENABLE_COVERAGE_TESTING)
    set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/test*")
    add_code_coverage(
      NAME coverage_report
      DEPENDENCIES ${PROJECT_NAME}_test
    )
  endif()
endif()

