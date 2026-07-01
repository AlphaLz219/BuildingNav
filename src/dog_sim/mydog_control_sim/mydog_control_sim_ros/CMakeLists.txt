cmake_minimum_required(VERSION 2.8.3)
project(mydog_control_sim_ros)

find_package(Boost REQUIRED COMPONENTS 
	thread 
	chrono
	atomic
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  trajectory_msgs
  geometry_msgs
  tf
  std_srvs
  joy
  controller_manager
  dynamic_reconfigure
)

find_package(Eigen3 REQUIRED)

set(MNN_INCLUDE_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../MNN/include")
set(MNN_LIBRARIES "${CMAKE_CURRENT_LIST_DIR}/../../MNN/build/libMNN.so")

catkin_package(
  INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIR}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS 
    roscpp 
    std_msgs 
    trajectory_msgs 
    geometry_msgs 
    tf
    std_srvs
    joy
    controller_manager
  DEPENDS
    Boost
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIRS}
  ${MNN_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
   src/${PROJECT_NAME}/MydogControlSimRos.cpp
 )

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(${PROJECT_NAME}_node
  src/${PROJECT_NAME}_node.cpp
)

add_dependencies(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

target_include_directories(${PROJECT_NAME}_node PRIVATE
  include
)
target_include_directories(${PROJECT_NAME}_node SYSTEM PUBLIC
  ${catkin_INCLUDE_DIRS}
)

## Specify libraries to link a library or executable target against
 target_link_libraries(${PROJECT_NAME}_node
   ${PROJECT_NAME}
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
   ${MNN_LIBRARIES}
 )

#############
## Install ##
#############

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
 install(TARGETS ${PROJECT_NAME}
 	${PROJECT_NAME}_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
 )

## Mark cpp header files for installation
 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
 )

## Mark other files for installation (e.g. launch andiles/modeling.dir/al bag files, etc.)
install(
  DIRECTORY
    config
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

