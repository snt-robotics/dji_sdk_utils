cmake_minimum_required(VERSION 2.8.3)
project(att_quat_to_transform)

find_package(catkin REQUIRED COMPONENTS
  rospy
  tf
  tf2_ros
  geometry_msgs
  std_msgs
  dji_sdk
)

catkin_python_setup()
