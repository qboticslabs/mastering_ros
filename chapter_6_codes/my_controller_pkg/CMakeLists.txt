cmake_minimum_required(VERSION 2.8.3)
project(my_controller_pkg)

find_package(catkin REQUIRED COMPONENTS
  pluginlib
  pr2_controller_interface
  pr2_mechanism_model
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)

## Define catkin exports
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES my_controller_pkg
  CATKIN_DEPENDS pluginlib roscpp
  DEPENDS Boost
)


include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

## pluginlib_tutorials library
add_library(my_controller_lib src/my_controller_file.cpp)
target_link_libraries(my_controller_lib ${catkin_LIBRARIES})


#install(TARGETS my_controller_lib
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.h"
#  PATTERN ".svn" EXCLUDE)

## Mark other files for installation (e.g. launch and bag files, etc.)
#install(FILES calculator_plugins.xml
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

