cmake_minimum_required(VERSION 3.0.2)
project(autonomous_navigation)

find_package(catkin REQUIRED)

catkin_package()

## Scripts: install Python executables that import catkin message bindings
catkin_install_python(PROGRAMS
  scripts/pointcloud_converter.py
  scripts/frontier_exploration.py
  scripts/frontier_exploration_a1.py
  scripts/cmd_vel_odom.py
  scripts/tf_to_odom.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
)
