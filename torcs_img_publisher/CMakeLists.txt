cmake_minimum_required(VERSION 2.8.3)
project(torcs_img_publisher)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  cv_bridge
  image_geometry
  image_transport
)

if(OpenCV_VERSION VERSION_LESS "3.0")
  # use 2.4 modules
else()
  # use 3.x modules
endif()

catkin_package()

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(torcs_img_publisher_node src/torcs_img_publisher.cpp)
target_link_libraries(torcs_img_publisher_node 
  ${catkin_LIBRARIES} 
)
add_dependencies(torcs_img_publisher_node ${catkin_EXPORTED_TARGETS})

