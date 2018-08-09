cmake_minimum_required(VERSION 2.8.3)
project(dji_camera)

find_package(catkin REQUIRED COMPONENTS
  dji_sdk
  robot_geometry_msgs
  geometry_msgs
  tf2
  tf2_ros
)
