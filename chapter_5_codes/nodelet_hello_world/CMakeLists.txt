cmake_minimum_required(VERSION 2.8.3)
project(nodelet_hello_world)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nodelet
  roscpp
  std_msgs
)


include_directories(${catkin_INCLUDE_DIRS})


catkin_package(
  LIBRARIES nodelet_hello_world
  CATKIN_DEPENDS nodelet roscpp std_msgs
)


## Declare a cpp library
 add_library(nodelet_hello_world
   src/hello_world.cpp
 )

## Specify libraries to link a library or executable target against
 target_link_libraries(nodelet_hello_world
   ${catkin_LIBRARIES}
 )

if(catkin_EXPORTED_LIBRARIES)
  add_dependencies(nodelet_hello_world ${catkin_EXPORTED_LIBRARIES})
endif()



# install the include folder
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES hello_world.xml hello_world.launch 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
