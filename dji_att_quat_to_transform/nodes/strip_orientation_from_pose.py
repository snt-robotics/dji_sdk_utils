#!/usr/bin/env python
"""
Strictly for testing purposes.

Subscribes to /DJI/pose OptiTrack topic and republishes the
/map -> /base_link without orientation such that we can apply
the absolute orientation transform from the AttitudeQuaternion
message
"""

import rospy
import tf, tf2_ros
from std_msgs.msg import Header
from geometry_msgs.msg import (
  TransformStamped,
  Transform,
  PoseStamped,
  Quaternion
)

NODE_NAME = 'strip_orientation_from_pose'

map_frame_id = None
baselink_translation_frame_id = None

def dji_pose_stamped_callback(msg):
  """
  Strip orientation from pose stamped  
  """

  if not map_frame_id:
    rospy.logwarn('%s: ~map_frame_id is not defined!', NODE_NAME)
    return

  if not baselink_translation_frame_id:
    rospy.logwarn('%s: ~baselink_translation_frame_id is not defined!', NODE_NAME)
    return

  #time = rospy.Time.now()
  time = msg.header.stamp

  h = Header()
  h.stamp = time
  h.frame_id = map_frame_id

  t_stamped = TransformStamped()
  t_stamped.header = h
  t_stamped.child_frame_id = baselink_translation_frame_id
  t_stamped.transform = Transform()
  t_stamped.transform.translation = msg.pose.position 
  t_stamped.transform.rotation = Quaternion(0, 0, 0, 1) 
  tf_broadcaster.sendTransform(t_stamped)

rospy.init_node(NODE_NAME)
rospy.Subscriber('/DJI/pose', PoseStamped, dji_pose_stamped_callback)
tf_broadcaster = tf2_ros.TransformBroadcaster()

map_frame_id = rospy.get_param('~map_frame_id')
baselink_translation_frame_id = rospy.get_param('~baselink_translation_frame_id')

rospy.loginfo('%s: Assuming map frame id to be %s', NODE_NAME, map_frame_id)
rospy.loginfo('%s: Assuming baselink translation (no orientation) frame id to be %s', NODE_NAME, baselink_translation_frame_id)

rospy.spin()
