cmake_minimum_required(VERSION 3.0.2)
project(whycon)

# set some flags
add_definitions(-std=c++11)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_RELEASE} -Wall -g -O4 -march=native -Wfatal-errors")
#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS_RELEASE} -Wall -O0 -g -Wfatal-errors")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake-configs")

# set some options
option(DISABLE_ROS "Do not build for ROS, but as standalone code" OFF)
option(ENABLE_FULL_UNDISTORT "Undistort the whole frame" OFF)
option(ENABLE_RANDOMIZED_THRESHOLD "Use rand() instead of binary-like search for threshold" OFF)
option(ENABLE_VERBOSE "Enable verbose console messages during detection" OFF)

#### ROS CONFIGURATION ####
if(NOT DISABLE_ROS)
  find_package(Eigen3 REQUIRED)
  find_package(catkin REQUIRED COMPONENTS
    cv_bridge
    roscpp
    sensor_msgs
    std_msgs
    camera_info_manager
    message_generation
    geometry_msgs
    message_filters
    image_geometry
    image_transport
    angles
    visualization_msgs
    tf
    nodelet
    std_srvs
    marker_planner
  )

  add_message_files(
    FILES
    Projection.msg
  )

  generate_messages(
    DEPENDENCIES geometry_msgs
  )

  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES whycon
    CATKIN_DEPENDS cv_bridge roscpp sensor_msgs std_msgs message_runtime geometry_msgs marker_planner
  )
endif()

set(_opencv_version 4)
find_package(OpenCV 4 REQUIRED )
if(NOT OpenCV_FOUND)
  message(STATUS "Did not find OpenCV 4, trying OpenCV 3")
  set(_opencv_version 3)
endif()

find_package(Boost COMPONENTS program_options thread system REQUIRED)

find_package(PkgConfig)
pkg_check_modules(YAML_CPP yaml-cpp)

### TARGETS ###
include_directories(
  ${catkin_INCLUDE_DIRS} ${YAML_CPP_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} include
)

add_library(whycon SHARED src/lib/circle_detector.cpp src/lib/many_circle_detector.cpp src/lib/localization_system.cpp)
target_link_libraries(whycon ${OpenCV_LIBS} ${Boost_LIBRARIES})

add_executable(whycon-node src/ros/whycon_node.cpp src/ros/whycon_ros.cpp)
set_target_properties(whycon-node PROPERTIES OUTPUT_NAME whycon)
add_library(whycon_nodelet src/ros/whycon_nodelet.cpp src/ros/whycon_ros.cpp)

add_dependencies(whycon-node whycon_generate_messages_cpp)
add_dependencies(whycon_nodelet whycon_generate_messages_cpp)

target_link_libraries(whycon-node ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES} whycon)
target_link_libraries(whycon_nodelet ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES} whycon)


### INSTALL ###

install(DIRECTORY include/${PACKAGE_NAME}
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
## Mark executables and/or libraries for installation
install(FILES nodelets.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
