cmake_minimum_required(VERSION 3.0.2)
project(dog_hybrid_planner)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
  tf
  gazebo_msgs
  tf2_ros
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs rospy sensor_msgs std_msgs tf gazebo_msgs tf2_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/dog_navigation.py
  scripts/improved_astar.py
  scripts/omni_dwa.py
  scripts/gt_pose_bridge.py
  scripts/fake_scan_from_map.py
  scripts/dog_kinematic_driver.py
  scripts/publish_initial_pose.py
  scripts/dog_command_bridge.py
  scripts/benchmark_omni_dwa.py
  scripts/benchmark_dog_astar.py
  scripts/run_navigation_experiment.py
  scripts/plot_navigation_experiment.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config maps worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
