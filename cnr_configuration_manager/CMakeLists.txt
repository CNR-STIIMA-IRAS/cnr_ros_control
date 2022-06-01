cmake_minimum_required(VERSION 3.1)

project(cnr_configuration_manager)

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/cnrConfigMacros.cmake)

cnr_set_flags()

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  pluginlib
  rosconsole
  configuration_msgs
  control_msgs
  cnr_controller_manager_interface
  cnr_controller_interface
  cnr_hardware_driver_interface
  cnr_hardware_interface
  controller_manager
  hardware_interface
  actionlib
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
  CATKIN_DEPENDS  cnr_controller_manager_interface  cnr_controller_interface cnr_hardware_driver_interface 
                  cnr_hardware_interface cnr_logger configuration_msgs actionlib control_msgs controller_manager 
                  hardware_interface configuration_msgs subscription_notifier pluginlib rosconsole roscpp
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
target_link_libraries (${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
cnr_target_compile_options(${PROJECT_NAME})

add_executable        (${PROJECT_NAME}_node src/configuration_manager_node.cpp )
add_dependencies      (${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries (${PROJECT_NAME}_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})
cnr_target_compile_options(${PROJECT_NAME}_node)

if(${CMAKE_VERSION} VERSION_GREATER "3.16.0")
	target_precompile_headers (${PROJECT_NAME} PUBLIC <ros/ros.h>
													  <controller_manager_msgs/LoadController.h>
													  <controller_manager_msgs/UnloadController.h>
													  <controller_manager_msgs/SwitchController.h>
													  <controller_manager_msgs/ListControllers.h>
													  <nodelet/nodelet.h>
													  <nodelet/loader.h>
													  <nodelet/NodeletLoadRequest.h>
													  <nodelet/NodeletLoad.h>
													  <nodelet/NodeletUnload.h>
													  <nodelet/NodeletList.h>
													  <std_srvs/Trigger.h>
													  <boost/graph/graph_concepts.hpp>
													  <actionlib/client/simple_action_client.h>
													  <control_msgs/FollowJointTrajectoryAction.h>
													  <bondcpp/bond.h>
													  <mutex>
													  <string> 
													  <stdexcept>)
endif()
#target_compile_definitions(${PROJECT_NAME}_node PUBLIC  $<$<CONFIG:RELEASE>:NDEBUG> -fpermissive)

add_executable(configuration_user_interface src/ui_configuration_manager.cpp)
add_dependencies(configuration_user_interface ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(configuration_user_interface ${catkin_LIBRARIES})
cnr_target_compile_options(configuration_user_interface)

add_executable(joy_teach_pendant src/joy_teach_pendant.cpp)
add_dependencies(joy_teach_pendant ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(joy_teach_pendant ${catkin_LIBRARIES})
cnr_target_compile_options(joy_teach_pendant)

add_executable(dispatcher src/dispatcher.cpp)
add_dependencies(dispatcher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dispatcher ${catkin_LIBRARIES})
cnr_target_compile_options(dispatcher)


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
    set(COVERAGE_EXCLUDES "*/test*")
    add_code_coverage(
      NAME coverage_report
      DEPENDENCIES ${PROJECT_NAME}_test
    )
  endif()
endif()


