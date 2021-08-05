cmake_minimum_required(VERSION 3.1.2)
project(go_back_recovery)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  tf2
  tf2_geometry_msgs
  tf2_ros
  angles
  costmap_2d
  nav_core
  pluginlib
  base_local_planner
  roscpp
)

find_package(Eigen REQUIRED)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES go_back_recovery
  CATKIN_DEPENDS roscpp pluginlib costmap_2d geometry_msgs nav_core tf2 tf2_ros
#  DEPENDS system_lib
)

add_library(go_back_recovery src/go_back_recovery.cpp)
target_link_libraries(go_back_recovery ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
 )

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )

 install(FILES
    recovery_plugin.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
 )
