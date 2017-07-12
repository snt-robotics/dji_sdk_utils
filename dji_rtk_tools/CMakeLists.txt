cmake_minimum_required(VERSION 2.8.3)
project(dji_rtk_tools)

find_package(catkin REQUIRED COMPONENTS
  rospy
  dji_sdk
  std_msgs
  std_srvs
  sensor_msgs
  geometry_msgs
  rospy
  tf2_ros
  tf
)

catkin_python_setup()
