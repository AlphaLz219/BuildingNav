cmake_minimum_required(VERSION 3.0.2)
project(autonomous_navigation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf
  tf2_ros
  urdf
  xacro
  gazebo_ros
  gazebo_plugins
  slam_gmapping
  explore_lite
  map_server
  move_base
  costmap_2d
  base_local_planner
  dwa_local_planner
  amcl
  pointcloud_to_laserscan
)

catkin_package(
  INCLUDE_DIRS 
  LIBRARIES 
  CATKIN_DEPENDS 
    roscpp 
    rospy 
    std_msgs 
    geometry_msgs 
    nav_msgs 
    sensor_msgs 
    tf 
    tf2_ros 
    urdf 
    xacro
    gazebo_ros
    gazebo_plugins
  DEPENDS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/turtlebot3_controller.py
  scripts/exploration_manager.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(DIRECTORY urdf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
)

install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
)

install(DIRECTORY maps/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/maps
)
