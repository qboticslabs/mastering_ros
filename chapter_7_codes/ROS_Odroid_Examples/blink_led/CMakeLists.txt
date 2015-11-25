cmake_minimum_required(VERSION 2.8.3)
project(blink_led)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

find_package(Boost REQUIRED COMPONENTS system)


include_directories(
  ${catkin_INCLUDE_DIRS}
  "/usr/local/include"
)

#INCLUDE_DIRECTORIES("/usr/local/include")
LINK_DIRECTORIES("/usr/local/lib")



add_executable(blink_led_node src/blink.cpp)

target_link_libraries(blink_led_node
   ${catkin_LIBRARIES} wiringPi
 )



