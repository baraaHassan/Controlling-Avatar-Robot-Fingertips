cmake_minimum_required(VERSION 3.0.2)
project(hand_ik)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  sense_glove
  tf
  geometry_msgs
  visualization_msgs
  eigen_conversions
  moveit_core
  moveit_ros_planning
  tf_conversions
  interactive_markers
  config_server
)

find_package(Eigen3 REQUIRED)

catkin_package()

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(Publisher src/Publisher.cpp)
target_link_libraries(Publisher ${catkin_LIBRARIES})

add_executable(hand_controller src/hand_controller.cpp src/ik.cpp)
target_link_libraries(hand_controller ${catkin_LIBRARIES})

##add_executable(ik src/ik.cpp)
##target_link_libraries(ik ${catkin_LIBRARIES})

add_executable(hand_ik src/hand_ik.cpp)
target_link_libraries(hand_ik ${catkin_LIBRARIES})
