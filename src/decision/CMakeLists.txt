cmake_minimum_required(VERSION 3.0.2)
project(decision)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  morai_msgs
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES decision
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

# Specify where the Python scripts are located
catkin_install_python(
  PROGRAMS
    src/pure_pursuit.py
    src/mode_decider.py
    src/mode_decider_Morai.py
    src/pure_pursuit_Henes.py
    src/pure_pursuit_Morai.py
    src/path_maker.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)