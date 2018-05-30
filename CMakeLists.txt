cmake_minimum_required(VERSION 2.8.3)
project(robots_moviles)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  nav_msgs
  move_base_msgs
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES badger_gazebo
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)
include_directories(
# include
 ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS}
 #${DYNAMICEDT3D_INCLUDE_DIRS} ${OCTOMAP_INCLUDE_DIRS} ${VORO++_INCLUDE_DIRS}
)

 install(PROGRAMS
   scripts/explore.py
   scripts/measure_distance_time.py
   scripts/csv_to_map.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

