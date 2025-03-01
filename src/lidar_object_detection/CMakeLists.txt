cmake_minimum_required(VERSION 3.0.2)
project(lidar_object_detection)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  geometry_msgs
  nav_msgs
  message_generation
  laser_geometry
  dynamic_reconfigure
)

add_message_files(
   FILES
   Boundingbox.msg
   ObjectInfo.msg
   PointInfo.msg
   NearestObstacleInfo.msg
)


generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
  geometry_msgs
  nav_msgs
)


generate_dynamic_reconfigure_options(
  cfg/config_acc.cfg
  cfg/config_without_clustering.cfg
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES lidar_object_detection
  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp sensor_msgs std_msgs geometry_msgs nav_msgs message_runtime laser_geometry dynamic_reconfigure
#  EXPORTED_TARGETS ${PROJECT_NAME}_generate_message_cpp
  DEPENDS system_lib
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(object_detection_acc src/object_detection_acc.cpp)
target_link_libraries(object_detection_acc ${catkin_LIBRARIES})
add_dependencies(object_detection_acc ${PROJECT_NAME}_gencfg)

add_executable(od_without_clustering src/od_without_clustering.cpp)
target_link_libraries(od_without_clustering ${catkin_LIBRARIES})
add_dependencies(od_without_clustering ${PROJECT_NAME}_gencfg)

# Specify where the Python scripts are located
catkin_install_python(PROGRAMS
    scripts/lattice_basic.py
    scripts/occupancy_grid_mapper.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)