cmake_minimum_required(VERSION 3.0.2)
project(autonomous_navigation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf
  tf2
  tf2_ros
  urdf
  xacro
  gazebo_ros
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
