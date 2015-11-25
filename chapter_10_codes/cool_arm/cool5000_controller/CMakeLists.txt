cmake_minimum_required(VERSION 2.8.3)
project(cool5000_controller)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  rospy 
  std_msgs
  actionlib
  dynamixel_msgs
  sensor_msgs
  tf
)

# Declare a catkin package
catkin_package(
  CATKIN_DEPENDS 
    roscpp 
    rospy 
    std_msgs
    actionlib 
    std_msgs 
    sensor_msgs
    dynamixel_msgs
    tf
)

## Build 
include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
)

# ---------------
add_executable(joint_state_aggregator src/joint_state_aggregator.cpp)
target_link_libraries(joint_state_aggregator ${catkin_LIBRARIES} )
