cmake_minimum_required(VERSION 3.14.0)
project(controller_msgs)

find_package(catkin REQUIRED COMPONENTS geometry_msgs sensor_msgs message_generation std_msgs)

include_directories(include)


add_service_files(
  FILES
  setmode.srv
  getmode.srv
  start.srv
  stop.srv
)

add_message_files(
  DIRECTORY msg
  FILES
  FlatTarget.msg
  PositionCommand.msg
  Sequence.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS geometry_msgs sensor_msgs message_runtime std_msgs)