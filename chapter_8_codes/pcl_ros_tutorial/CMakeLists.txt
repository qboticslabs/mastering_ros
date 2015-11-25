cmake_minimum_required(VERSION 2.8.3)
project(pcl_ros_tutorial)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(pcl_publisher_node src/pcl_publisher.cpp)
add_executable(pcl_filter src/pcl_filter.cpp)
add_executable(pcl_write src/pcl_write.cpp)
add_executable(pcl_read src/pcl_read.cpp)

target_link_libraries(pcl_publisher_node
   ${catkin_LIBRARIES}
 )

target_link_libraries(pcl_filter
   ${catkin_LIBRARIES}
 )
target_link_libraries(pcl_write
   ${catkin_LIBRARIES}
 )
target_link_libraries(pcl_read
   ${catkin_LIBRARIES}
 )

