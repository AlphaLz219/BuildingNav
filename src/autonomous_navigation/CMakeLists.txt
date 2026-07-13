cmake_minimum_required(VERSION 3.0.2)
project(autonomous_navigation)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

add_message_files(
  FILES
  CustomPoint.msg
  CustomMsg.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)

# 安装 Python 脚本到 devel/install 空间，确保 roslaunch 能找到消息绑定
catkin_install_python(PROGRAMS
  scripts/pointcloud_converter.py
  scripts/pointcloud_to_custommsg.py
  scripts/pointcloud_to_2d_map.py
  scripts/tf_bridge.py
  scripts/lio_to_base.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
