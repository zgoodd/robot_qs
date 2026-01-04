cmake_minimum_required(VERSION 2.8.3)
project(xleg_controller)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


find_package(catkin REQUIRED)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  dynamixel_workbench_msgs
  dynamixel_workbench_toolbox
  cmake_modules
  dynamixel_sdk

  std_msgs
  serial
  message_generation
 ## xleg_lowersys

 OpenCV
 cv_bridge
 image_transport
)

find_package(Eigen3 REQUIRED)
find_package(Qt5
REQUIRED COMPONENTS
Widgets
)

find_package(OpenCV REQUIRED)      #vision


set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)


find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIR
  NAMES yaml_cpp.h
  PATHS ${YAML_CPP_INCLUDE_DIRS}
)
find_library(YAML_CPP_LIBRARY
  NAMES YAML_CPP
  PATHS ${YAML_CPP_LIBRARY_DIRS}
)
link_directories(${YAML_CPP_LIBRARY_DIRS})

if(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")
add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")


catkin_package(
#  INCLUDE_DIRS include
 CATKIN_DEPENDS 
 roscpp 
 sensor_msgs geometry_msgs 
 dynamixel_workbench_msgs trajectory_msgs 
 dynamixel_workbench_toolbox cmake_modules
 DEPENDS EIGEN3
 serial
 std_msgs
)


include_directories(
  include ${catkin_INCLUDE_DIRS}
    src
    src/ui
    include 
    ${catkin_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/..        # 为了将ui文件单独放，后加的
)

include_directories(
  ${catkin_INCLUDE_DIRS}
	 ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_BINARY_DIR}
)

include_directories(${OpenCV_INCLUDE_DIRS})

SET(my_HEADERS
  src/mainwindow.h
  # src/qnode.h
  src/motionctrldlg.h
  src/messagethread.h
  src/testdirectctrldlg.h
)

set(my_SOURCES
  src/mainwindow.cpp
  src/motionctrldlg.cpp
  src/messagethread.cpp
  src/testdirectctrldlg.cpp
)

set(my_UI
  # src/ui/mainwindow.ui
  # src/ui/motionctrldlg.ui
  # src/ui/testdirectctrldlg.ui
    src/mainwindow.ui
    src/motionctrldlg.ui
    src/testdirectctrldlg.ui
)

SET(my_all_SOURCES
    ${my_SOURCES}
    ${my_UI}
    ${my_HEADERS}
#    ${my_MOC_RESOURCES}
)


# # 为了将ui文件单独放，后加的
# QT5_WRAP_UI(my_MOC_UI
#     ${my_UI}
# )
# QT5_WRAP_CPP(my_MOC_HEADERS
#     ${my_HEADERS}
# )
# QT5_ADD_RESOURCES(my_MOC_RESOURCES
#    ${my_RESOURCES}
#  )
# SET(my_all_SOURCES
#     ${my_SOURCES}
#     ${my_MOC_UI}
#     ${my_MOC_HEADERS}
# #    ${my_MOC_RESOURCES}
# )




# ${CMAKE_CURRENT_SOURCE_DIR}:/home/xzy/x_robot/src/xleg_controller
# file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/*.ui) 
# file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
# file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS
# include/robot_gui/*.hpp) QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})

# QT5_WRAP_UI(UIC ${my_UI} ) 
# QT4_WRAP_CPP(QT_MOC_HPP ${QT_MOC})




# add_executable(xleg_robot  ${my_SOURCES} ${my_HEADERS} ${my_UI} )
add_executable(xleg_robot  ${my_all_SOURCES} )
target_link_libraries(xleg_robot  Qt5::Widgets ${catkin_LIBRARIES})

#  第二个node
add_executable(test_single_motor        src/test_single_motor.cpp )
add_dependencies(test_single_motor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test_single_motor     ${catkin_LIBRARIES} )


add_executable(process_cam
  src/process_cam.cpp
)
add_dependencies(process_cam ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(process_cam
  ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
)


add_executable(test
  src/test.cpp
)
add_dependencies(test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(test
  ${catkin_LIBRARIES}
)
