cmake_minimum_required(VERSION 3.0.2)
project(pioneer)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  gazebo_ros
  gazebo_ros_control
  roscpp
  rospy
  std_msgs
  urdf
  xacro
)

find_package(gazebo REQUIRED)
find_package (Eigen3 REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/pioneer_plugin.cc 
                            src/Walkingpattern_generator.cc
                            src/Wonbin_Walkingpattern_generator.cc
                            )

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})


## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  DEPENDS
  roscpp
  rospy
  gazebo_ros
#  INCLUDE_DIRS include
#  LIBRARIES testbot10_gazebo
#  CATKIN_DEPENDS controller_manager gazebo_ros gazebo_ros_control roscpp std_msgs urdf xacro
#  DEPENDS system_lib
)


## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

