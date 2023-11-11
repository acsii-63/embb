cmake_minimum_required(VERSION 3.14.0)
project(sequence_controller)

add_definitions(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  dynamic_reconfigure
  tf
  mavros
  mavros_extras
  mavros_msgs
  mavlink
  controller_msgs
  message_filters
  message_generation
  yaml_cpp_catkin
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES sequence_controller
  CATKIN_DEPENDS roscpp rospy std_msgs mavros_msgs geometry_msgs sensor_msgs tf  message_filters controller_msgs yaml_cpp_catkin
)

#############
# LIBRARIES #
#############
include_directories(
  include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

############
# BINARIES #
############

add_executable(parser
  src/parser.cpp
)

add_executable(opt
  src/arucopt.cpp
)

target_link_libraries(parser ${catkin_LIBRARIES})


target_link_libraries(opt ${catkin_LIBRARIES})
