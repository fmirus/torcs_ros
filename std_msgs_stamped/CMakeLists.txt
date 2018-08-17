cmake_minimum_required(VERSION 2.8.3)
project(std_msgs_stamped)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)


## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Int8Stamped.msg
  Float32Stamped.msg
  BoolStamped.msg
  UInt32Stamped.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS std_msgs message_runtime)


