cmake_minimum_required(VERSION 3.5.1)
project(rqt_rviz_wrapper)

set(CMAKE_CXX_STANDARD 11)
add_definitions(-Wall -Wextra -Wpedantic)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CATKIN_PACKAGE_DEPENDENCIES
  rqt_gui
  rqt_gui_cpp
  spinner_widget
  signal_handler
  
)

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_PACKAGE_DEPENDENCIES}
)

# Find wmctrl
find_program(WMCTRL
  NAMES
    wmctrl
  PATHS
    /usr/bin
    /usr/local/bin
)
if(NOT WMCTRL)
  message(FATAL_ERROR "wmctrl not found. Install wmctrl e.g. sudo apt-get install wmctrl")
endif()

find_package(Qt5 COMPONENTS Core Widgets REQUIRED)

set(rqt_rviz_wrapper_SRCS
  src/rqt_rviz_wrapper/RvizWrapper.cpp
)

set(rqt_rviz_wrapper_HDRS_QT
  include/rqt_rviz_wrapper/RvizWrapper.h
)

set(rqt_rviz_wrapper_HDRS
)

set(rqt_rviz_wrapper_UIS
  src/rqt_rviz_wrapper/RvizWrapper.ui
)

set(ui_INCLUDE_DIR
  "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_INCLUDE_DESTINATION}/${PROJECT_NAME}"
)

set(rqt_rviz_wrapper_INCLUDE_DIRECTORIES
  include
  SYSTEM
    ${ui_INCLUDE_DIR}/..
)

if(NOT EXISTS ${ui_INCLUDE_DIR})
  file(MAKE_DIRECTORY ${ui_INCLUDE_DIR})
endif()

catkin_package(
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
)

include_directories(
  ${rqt_rviz_wrapper_INCLUDE_DIRECTORIES}
  SYSTEM
    ${catkin_INCLUDE_DIRS}
)

qt5_wrap_cpp(rqt_rviz_wrapper_MOCS ${rqt_rviz_wrapper_HDRS_QT})

# ensure generated header files are being created in the devel space
set(_cmake_current_binary_dir "${CMAKE_CURRENT_BINARY_DIR}")
set(CMAKE_CURRENT_BINARY_DIR ${ui_INCLUDE_DIR})
qt5_wrap_ui(rqt_rviz_wrapper_UIS_H ${rqt_rviz_wrapper_UIS})
set(CMAKE_CURRENT_BINARY_DIR "${_cmake_current_binary_dir}")

add_library(rqt_rviz_wrapper
  ${rqt_rviz_wrapper_SRCS}
  ${rqt_rviz_wrapper_HDRS_QT}
  ${rqt_rviz_wrapper_HDRS}
  ${rqt_rviz_wrapper_MOCS}
  ${rqt_rviz_wrapper_UIS_H}
)
qt5_use_modules(rqt_rviz_wrapper
  Widgets
)
target_link_libraries(rqt_rviz_wrapper
  Qt5::Core
  Qt5::Widgets
  ${catkin_LIBRARIES}
)

find_package(class_loader)
class_loader_hide_library_symbols(rqt_rviz_wrapper)

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING
  PATTERN "*.launch"
)
install(TARGETS rqt_rviz_wrapper
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(PROGRAMS scripts/rqt_rviz_wrapper
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/rqt_rviz_wrapper/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
  add_default_clang_tooling()
endif(cmake_clang_tools_FOUND)

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
