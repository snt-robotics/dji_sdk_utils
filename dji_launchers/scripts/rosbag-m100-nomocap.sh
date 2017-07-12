#!/bin/bash
source ./record.sh

BUFFER_SIZE=512
TOPICS="/dji_sdk/acceleration /dji_sdk/attitude_quaternion /dji_sdk/compass /dji_sdk/global_position /dji_sdk/local_position /dji_sdk/odometry /dji_sdk/velocity /dji_sdk/image_raw /tf /tf_static"

record_topics "\"${TOPICS}\"" "${BUFFER_SIZE}" $1
