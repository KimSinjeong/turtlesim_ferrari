cmake_minimum_required(VERSION 3.0.2)
project(turtlesim_ferrari)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  rosserial_python
  sensor_msgs
  std_msgs
  pcl_ros
  ndt_omp
)

find_package(PCL REQUIRED)

catkin_package(
#  LIBRARIES turtlesim_ferrari
  CATKIN_DEPENDS roscpp rospy rosserial_python sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(keyboard_controller src/ferrari_control.cpp src/key_input_fu.cpp)
add_dependencies(keyboard_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(keyboard_controller
  ${catkin_LIBRARIES}
#  turtlesim_ferrari
)

add_executable(track_extractor src/track_extractor.cpp)
add_dependencies(track_extractor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(track_extractor
  ${catkin_LIBRARIES}
)

add_executable(scan_matcher src/localizer.cpp)
add_dependencies(scan_matcher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(scan_matcher
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)