cmake_minimum_required(VERSION 3.1)

project(cnr_hardware_interface)


if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  cnr_logger
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

if(CATKIN_ENABLE_TESTING AND ENABLE_COVERAGE_TESTING)
  find_package(code_coverage REQUIRED)
  # Add compiler flags for coverage instrumentation before defining any targets
  APPEND_COVERAGE_COMPILER_FLAGS()
endif()

catkin_package(
 INCLUDE_DIRS   include
 LIBRARIES      cnr_hardware_interface
 CATKIN_DEPENDS cnr_logger
                controller_manager
                diagnostic_updater
                diagnostic_msgs
                hardware_interface
                nodelet roscpp
                sensor_msgs
                configuration_msgs
                realtime_utilities
                pluginlib
)

include_directories   (include include/internal ${catkin_INCLUDE_DIRS} )
add_library           (${PROJECT_NAME} src/${PROJECT_NAME}/cnr_robot_hw.cpp )
add_dependencies      (${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries (${PROJECT_NAME} ${catkin_LIBRARIES})
target_compile_options(${PROJECT_NAME} PUBLIC -Wall -faligned-new
        $<$<CONFIG:Release>:-Ofast -funroll-loops -ffast-math >)

set(ROSLINT_CPP_OPTS "--filter=-runtime/references,-runtime/int,-build/header_guard --linelength=150")
roslint_cpp(src/${PROJECT_NAME}/cnr_robot_hw.cpp include/${PROJECT_NAME}/cnr_robot_hw.h)

#############
## Install ##
#############

## Mark cpp header files for installation
install( DIRECTORY      include/${PROJECT_NAME}/
         DESTINATION    ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
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
