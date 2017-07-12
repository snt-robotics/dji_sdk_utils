#!/bin/bash
source ./record.sh

BUFFER_SIZE=1024
TOPICS=(/dji_sdk/A3_RTK /dji_sdk/A3_GPS /dji_sdk/acceleration /dji_sdk/attitude_quaternion /dji_sdk/gimbal /dji_sdk/gimbal_joint_states /dji_sdk/compass /tf /tf_static /dji_sdk/initial_position)

record_topics "${TOPICS[@]}" "${BUFFER_SIZE}" $1
