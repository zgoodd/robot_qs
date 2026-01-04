cmake_minimum_required(VERSION 2.8.3)
project(xleg_rosbridge)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  xleg_msgs
  dynamic_reconfigure
  message_generation
)

## Generate dynamic reconfigure files
generate_dynamic_reconfigure_options(
  cfg/RosbridgeTopic.cfg
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS roscpp std_msgs xleg_msgs dynamic_reconfigure
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

###########
## Declare C++ executables ##
###########

## rosbridge话题管理器节点
add_executable(rosbridge_topic_manager
  src/rosbridge_topic_manager.cpp
)
add_dependencies(rosbridge_topic_manager ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rosbridge_topic_manager
  ${catkin_LIBRARIES}
)

###########
## Install ##
###########

