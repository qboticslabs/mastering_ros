cmake_minimum_required(VERSION 2.8.3)
project(moveit_simple_grasps)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  eigen_conversions
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_core
  moveit_visual_tools
  tf
  tf_conversions
  trajectory_msgs
  message_generation
  geometry_msgs
  actionlib
  actionlib_msgs
  moveit_msgs
  cmake_modules
)

find_package(Eigen REQUIRED)
find_package(Boost REQUIRED thread system)

add_message_files(DIRECTORY msg FILES
  GraspGeneratorOptions.msg
)

add_action_files(DIRECTORY action FILES
  GenerateGrasps.action
)

generate_messages(DEPENDENCIES
  geometry_msgs 
  actionlib_msgs 
  std_msgs 
  moveit_msgs
)

# Catkin
catkin_package(
  LIBRARIES
    ${PROJECT_NAME}
    ${PROJECT_NAME}_filter
  CATKIN_DEPENDS
    actionlib_msgs
    geometry_msgs
    moveit_msgs
    trajectory_msgs
    std_msgs
    message_runtime
    moveit_visual_tools
  INCLUDE_DIRS include
)

## Build 
include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
)

# Grasp Generator Library
add_library(${PROJECT_NAME}
  src/simple_grasps.cpp
  src/grasp_data.cpp
)
target_link_libraries(${PROJECT_NAME} 
  ${catkin_LIBRARIES} ${Boost_LIBRARIES}
)

# Grasp Filter Library
add_library(${PROJECT_NAME}_filter
  src/grasp_filter.cpp
)
target_link_libraries(${PROJECT_NAME}_filter 
  ${catkin_LIBRARIES} ${Boost_LIBRARIES}
)

# Action Server executable
add_executable(${PROJECT_NAME}_server src/simple_grasps_server.cpp)
add_dependencies(${PROJECT_NAME}_server ${catkin_EXPORTED_TARGETS}) # don't build until necessary msgs are finish
target_link_libraries(${PROJECT_NAME}_server
  ${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES}
)

# Test filter executable
add_executable(${PROJECT_NAME}_filter_test src/grasp_filter_test.cpp)
target_link_libraries(${PROJECT_NAME}_filter_test
  ${PROJECT_NAME} ${PROJECT_NAME}_filter ${catkin_LIBRARIES} ${Boost_LIBRARIES}
)

# Test grasp executable
add_executable(${PROJECT_NAME}_test src/simple_grasps_test.cpp)
target_link_libraries(${PROJECT_NAME}_test 
  ${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES}
)

## Install

# Install libraries
install(TARGETS 
  ${PROJECT_NAME} 
  ${PROJECT_NAME}_filter 
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

# Install shared resources
install(DIRECTORY launch    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY resources DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# Install executables
install(TARGETS 
  ${PROJECT_NAME}_server
  ${PROJECT_NAME}_test
  ${PROJECT_NAME}_filter_test
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
