cmake_minimum_required(VERSION 3.0.2)
project(vision_process)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)



find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
std_msgs
serial
)

find_package(Qt5
REQUIRED COMPONENTS
Widgets
)

find_package(OpenGL REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS
  serial
  std_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include ${catkin_INCLUDE_DIRS}
# include
# ${catkin_INCLUDE_DIRS}
)

include_directories( src
    include ${catkin_INCLUDE_DIRS} )
    include_directories(${OpenCV_INCLUDE_DIRS}) 


add_executable(get_xlegrobot_position
  src/get_xlegrobot_position.cpp
)
add_dependencies(get_xlegrobot_position ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(get_xlegrobot_position
  ${catkin_LIBRARIES}
    realsense2  
  glfw
  ${OpenCV_LIBS}
  Qt5::Widgets
)



# machine vision
add_executable(test_realsense     src/test_realsense.cpp )
add_dependencies(test_realsense ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_realsense
  realsense2
  glfw
#  OpenGL::GL
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}  Qt5::Widgets)

add_executable(test_image_recognition     src/test_image_recognition.cpp )
add_dependencies(test_image_recognition ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_image_recognition
  realsense2
  glfw
#  OpenGL::GL
  ${OpenCV_LIBS}
  ${catkin_LIBRARIES}  Qt5::Widgets)
