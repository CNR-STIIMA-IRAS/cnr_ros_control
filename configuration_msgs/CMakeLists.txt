cmake_minimum_required(VERSION 3.9)

project(configuration_msgs)
add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

add_message_files(
  FILES
  ConfigurationComponent.msg
)

add_service_files(
  FILES
  StartConfiguration.srv
  StopConfiguration.srv
  ListConfigurations.srv
  SetConfig.srv
  GetConfig.srv
  UpdateConfigurations.srv
  SendMessage.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES configuration_msgs
 CATKIN_DEPENDS roscpp std_msgs 
#  DEPENDS 
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
