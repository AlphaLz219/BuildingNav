cmake_minimum_required(VERSION 3.5.1)
project(qcustomplot)

add_definitions(-std=c++11)
add_definitions(-DQCUSTOMPLOT_COMPILE_LIBRARY)

find_package(catkin REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)
find_package(Qt5PrintSupport REQUIRED)
find_package(Qt5Widgets REQUIRED)

set(${PROJECT_NAME}_HDRS
  include/${PROJECT_NAME}/qcustomplot.h
)
set(${PROJECT_NAME}_SRCS
  src/${PROJECT_NAME}/qcustomplot.cpp
)

catkin_package(
  INCLUDE_DIRS
    include
    ${Qt5Core_INCLUDE_DIRS}
    ${Qt5Gui_INCLUDE_DIRS}
    ${Qt5PrintSupport_INCLUDE_DIRS}
    ${Qt5Widgets_INCLUDE_DIRS}
  LIBRARIES
    ${PROJECT_NAME}
    Qt5::Core
    Qt5::Gui
    Qt5::PrintSupport
    Qt5::Widgets
  CFG_EXTRAS
    ${PROJECT_NAME}.cmake
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS}
  ${Qt5Gui_INCLUDE_DIRS}
  ${Qt5PrintSupport_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
)
add_definitions(${Qt5Core_DEFINITIONS})
add_definitions(${Qt5Gui_DEFINITIONS})
add_definitions(${Qt5PrintSupport_DEFINITIONS})
add_definitions(${Qt5Widgets_DEFINITIONS})
qt5_wrap_cpp(${PROJECT_NAME}_MOCS ${${PROJECT_NAME}_HDRS})

add_library(
  ${PROJECT_NAME}
  ${${PROJECT_NAME}_SRCS}
  ${${PROJECT_NAME}_MOCS}
)
add_dependencies(
  ${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  Qt5::Core
  Qt5::Gui
  Qt5::PrintSupport
  Qt5::Widgets
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

##########
## Test ##
##########
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_${PROJECT_NAME}
    test/EmptyTests.cpp
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/test
  )

  if(TARGET test_${PROJECT_NAME})
    target_link_libraries( test_${PROJECT_NAME}
      ${PROJECT_NAME}
      ${catkin_LIBRARIES}
      gtest_main
    )
    find_package(cmake_code_coverage QUIET)
    if(cmake_code_coverage_FOUND)
      add_gtest_coverage()
    endif(cmake_code_coverage_FOUND)
  endif()
endif()
