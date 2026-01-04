cmake_minimum_required(VERSION 2.8.3)
project(xleg_lowersys)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  xleg_msgs
  # quadctrl_msgs

  # sensor_msgs
  geometry_msgs
  # trajectory_msgs
  dynamixel_workbench_msgs
  dynamixel_workbench_toolbox
  cmake_modules
  dynamixel_sdk

  serial
  # message_generation

  image_transport
  cv_bridge
  )

# vision
#find_package(OpenGL REQUIRED)
find_package(OpenCV REQUIRED)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)



catkin_package(
  INCLUDE_DIRS
#  LIBRARIES xleg_lowersys
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
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
include_directories(${OpenCV_INCLUDE_DIRS})

SET(my_HEADERS
  # src/CenterToLeg.h
  src/robot_function.h
  src/robot_thread.h
  # src/quadruped_gait.h

)

SET(my_SOURCES
    # src/main.cpp
    src/node.cpp
    #     src/CenterToLeg.cpp
# src/herkulex_drive.cpp
src/robot_function.cpp
src/robot_thread.cpp
# src/quadruped_gait.cpp
# src/serial.cpp
# src/adc_comm.cpp
)

SET(my_all_SOURCES
    ${my_SOURCES}
    ${my_HEADERS}
)


add_executable(xleg_lowersys
    ${my_all_SOURCES})
# add_executable(xleg_lowersys    src/node.cpp)
add_dependencies(xleg_lowersys ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(xleg_lowersys
  ${catkin_LIBRARIES}

  realsense2
  glfw
#  OpenGL::GL
  ${OpenCV_LIBS}
)




add_executable(pub_footforce
  src/pub_footforce.cpp
)
add_dependencies(pub_footforce ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pub_footforce
  ${catkin_LIBRARIES}
)


add_executable(pub_imu
  src/pub_imu.cpp
)
add_dependencies(pub_imu ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pub_imu
  ${catkin_LIBRARIES}
)

add_executable(pub_ir_sensor
  src/pub_ir_sensor.cpp
)
add_dependencies(pub_ir_sensor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pub_ir_sensor
  ${catkin_LIBRARIES}
)