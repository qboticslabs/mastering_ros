cmake_minimum_required(VERSION 2.8.3)
project(cv_bridge_tutorial_pkg)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  )

find_package(OpenCV REQUIRED)
catkin_package()
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(sample_cv_bridge_node src/sample_cv_bridge_node.cpp)
 target_link_libraries(sample_cv_bridge_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
 )


