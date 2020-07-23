cmake_minimum_required(VERSION 3.1)

project(cnr_configuration_manager)

set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Debug only, collects stats on how callbacks are doled out to worker threads
#add_definitions(-DNODELET_QUEUE_DEBUG)


find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  pluginlib
  rosconsole
  configuration_msgs
  control_msgs
  cnr_controller_manager_interface
  cnr_controller_interface
  cnr_hardware_nodelet_interface
  cnr_hardware_interface
  controller_manager
  hardware_interface
  actionlib
  nodelet
  roscpp
  configuration_msgs
  subscription_notifier
  cnr_logger
)

## Find Boost (only headers)
find_package(Boost REQUIRED)

## Find UUID libraries
find_package(UUID REQUIRED)

if(CATKIN_ENABLE_TESTING AND ENABLE_COVERAGE_TESTING)
  find_package(code_coverage REQUIRED)
  # Add compiler flags for coverage instrumentation before defining any targets
  APPEND_COVERAGE_COMPILER_FLAGS()
endif()


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cnr_configuration_manager
  CATKIN_DEPENDS  cnr_controller_manager_interface  cnr_controller_interface cnr_hardware_nodelet_interface 
                  cnr_hardware_interface cnr_logger configuration_msgs actionlib control_msgs controller_manager 
                  hardware_interface nodelet configuration_msgs subscription_notifier pluginlib rosconsole roscpp
  DEPENDS Boost
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${BOOST_INCLUDE_DIRS}
 ${UUID_INCLUDE_DIRS}
 )

add_library(${PROJECT_NAME} src/${PROJECT_NAME}/cnr_configuration_loader.cpp
                            src/${PROJECT_NAME}/cnr_configuration_manager.cpp
                            src/${PROJECT_NAME}/signal_handler.cpp )

add_executable       (${PROJECT_NAME}_node src/configuration_manager_node.cpp )
add_dependencies     (${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME} ${catkin_LIBRARIES} )

add_executable(configuration_user_interface src/ui_configuration_manager.cpp)
add_dependencies(configuration_user_interface ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(configuration_user_interface
  ${catkin_LIBRARIES}
)

add_executable(joy_teach_pendant src/joy_teach_pendant.cpp)
add_dependencies(joy_teach_pendant ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(joy_teach_pendant
  ${catkin_LIBRARIES}
)

add_executable(dispatcher src/dispatcher.cpp)
add_dependencies(dispatcher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dispatcher
  ${catkin_LIBRARIES}
)


#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".git" EXCLUDE
)

install(TARGETS ${PROJECT_NAME}_node ${PROJECT_NAME} configuration_user_interface joy_teach_pendant dispatcher
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(${PROJECT_NAME}_test test/test.launch test/test.cpp)
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME} ${catkin_LIBRARIES} ${rostest_LIBRARIES} )

  if(ENABLE_COVERAGE_TESTING)
    set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/test*")
    add_code_coverage(
      NAME ${PROJECT_NAME}_coverage_report
      DEPENDENCIES ${PROJECT_NAME}_test
    )
  endif()
endif()


