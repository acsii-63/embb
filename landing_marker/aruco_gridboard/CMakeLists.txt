cmake_minimum_required(VERSION 3.14.0)
project(aruco_gridboard)

set(_opencv_version 4)
find_package(OpenCV 4 REQUIRED)
if(NOT OpenCV_FOUND)
  message(STATUS "Did not find OpenCV 4, trying OpenCV 3")
  set(_opencv_version 3)
endif()

find_package(OpenCV ${_opencv_version} REQUIRED
  COMPONENTS
    opencv_core
    opencv_imgproc
    opencv_imgcodecs
  CONFIG
)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_geometry
  geometry_msgs
  message_filters
  image_transport
  resource_retriever
  roscpp
  sensor_msgs
  std_msgs
  tf
  marker_planner
)



#find_package(Boost REQUIRED
#  COMPONENTS filesystem system signals regex date_time program_options thread)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)


catkin_package(
   CATKIN_DEPENDS
   cv_bridge
   geometry_msgs
   message_filters
   image_transport
   roscpp
   sensor_msgs
   std_msgs
   marker_planner
)

add_executable(aruco_gridboard
  src/main.cpp
  src/names.cpp
  src/names.h
  src/node.cpp
  src/node.h
)

target_link_libraries(aruco_gridboard
    ${resource_retriever_LIBRARIES}
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

add_executable(create_board
   src/create_board.cpp
)

target_link_libraries(create_board
   ${OpenCV_LIBS}
   ${OpenCV_LIBRARIES}
)

