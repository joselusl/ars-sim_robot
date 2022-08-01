cmake_minimum_required(VERSION 3.0.0)
project(ars_sim_robot)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  tf_conversions
  tf2_ros
)


catkin_package(
  DEPENDS
  CATKIN_DEPENDS rospy std_msgs sensor_msgs geometry_msgs tf_conversions tf2_ros ars_lib_helpers
  INCLUDE_DIRS
  LIBRARIES
)
