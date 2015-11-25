cmake_minimum_required(VERSION 2.8.3)
project(ros_wiring_examples)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

find_package(Boost REQUIRED COMPONENTS system)


set(wiringPi_include "/usr/local/include")


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${wiringPi_include}
)

LINK_DIRECTORIES("/usr/local/lib")


add_executable(blink_led src/blink.cpp)

add_executable(button_led src/button.cpp)

target_link_libraries(blink_led
   ${catkin_LIBRARIES} wiringPi
 )

target_link_libraries(button_led
   ${catkin_LIBRARIES} wiringPi
 )

