#!/bin/bash
source ./record.sh

BUFFER=512
TOPICS="/dji_sdk/acceleration /dji_sdk/attitude_quaternion /dji_sdk/compass /dji_sdk/global_position /dji_sdk/local_position /dji_sdk/odometry /dji_sdk/velocity /dji_sdk/image_raw /tf /tf_static"

record_topics $1 "${TOPICS}" "${BUFFER_SIZE}"
