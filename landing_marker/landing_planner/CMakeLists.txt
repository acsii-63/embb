cmake_minimum_required(VERSION 3.0.2)
project(marker_planner)

## Compile as C++17, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  controller_msgs
  geometric_controller
)

add_service_files(
  FILES
  terminate.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
 )

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs geometry_msgs nav_msgs controller_msgs geometric_controller
)

add_executable(${PROJECT_NAME}_node src/marker_planner.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})
