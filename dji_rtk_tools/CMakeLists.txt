cmake_minimum_required(VERSION 2.8.3)
project(dji_rtk_tools)

find_package(catkin REQUIRED COMPONENTS
  rospy
  dji_sdk
)

catkin_python_setup()
