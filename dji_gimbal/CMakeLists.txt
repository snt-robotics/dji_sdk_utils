cmake_minimum_required(VERSION 2.8.3)
project(dji_gimbal)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
  dji_sdk
)

catkin_python_setup()
