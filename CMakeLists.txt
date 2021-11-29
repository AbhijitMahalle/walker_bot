cmake_minimum_required(VERSION 3.0.2)
project(walker_bot)

## Compile as C++11, supported in ROS Melodic
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  tf
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES walker_bot
  CATKIN_DEPENDS roscpp rospy std_msgs tf
  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/walker_node.cpp src/obstacle_avoidance.cpp src/walker )
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})