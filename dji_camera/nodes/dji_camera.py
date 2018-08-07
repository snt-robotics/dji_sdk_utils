#!/usr/bin/env python
import rospy

from robot_geometry_msgs.msg import RobotState
from dji_sdk.srv import CameraAction

import tf2_ros
#
# camera_action = None
# source_frame_id = None
# target_frame_id = None
#
# output_filename = None
#
# tf_buffer = None
# tf_listener = None
#
# output_file = None

def waypoint_reached_callback(msg):
  waypoint_reached_time = msg.header.stamp
  before_calling_camera = rospy.Time.now()
  camera_client(0);
  after_calling_camera = rospy.Time.now()

  try:
    trans = tf_buffer.lookup_transform(source_frame_id, target_frame_id, waypoint_reached_time)
  except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    rospy.logwarn("Exception caught while getting transform")
    return

  print waypoint_reached_time, before_calling_camera, after_calling_camera

rospy.init_node("dji_camera")

tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)

source_frame_id = rospy.get_param("~source_frame_id")
target_frame_id = rospy.get_param("~target_frame_id")

output_filename = rospy.get_param("~output_filename")

camera_client = rospy.ServiceProxy('camera_action', CameraAction)

rospy.Subscriber('waypoint_reached', RobotState, waypoint_reached_callback)

with open(output_filename, 'wt') as output_file:
  rospy.spin()
