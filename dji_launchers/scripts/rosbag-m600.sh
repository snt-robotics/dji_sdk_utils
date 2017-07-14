#!/bin/bash

if [[ -z "${1}" ]]; then
  echo 'You must specify a directory to which rosbag will be saved!'
  exit
fi

echo 'Creating directory ...'
mkdir -p $1

if [ $? -eq 0 ]; then
  echo 'Launching rosbag...'
  rosbag record -b 1024 -o "${1}/" /dji_sdk/A3_RTK /dji_sdk/attitude_quaternion /dji_sdk/gimbal /dji_sdk/gimbal_joint_states /tf /tf_static /dji_sdk/initial_position
else
  echo 'Directory count not be created, do you have permissions ?'
fi

