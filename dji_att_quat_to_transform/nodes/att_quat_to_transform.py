#!/usr/bin/env python
"""
Converts the AttitudeQuaternion message into ROS REP103 standard
Publishes the message as a tf transform given frame ids, as well
as PoseStamped message

The orientation is given with respect to ENU frame!
OptiTrack Lab has to account for this and correct by rotating to North
"""

import rospy
import tf, tf2_ros
from dji_sdk.msg import AttitudeQuaternion
from std_msgs.msg import Header
from geometry_msgs.msg import (
  TransformStamped,
  Transform,
  PoseStamped,
  Pose,
  Quaternion, 
  Vector3,
  Point
)

NODE_NAME = 'att_quat_to_transform'

map_frame_id = None
baselink_translation_frame_id = None

def attitude_quaternion_callback(msg):
  """
  DJI SDK defines quaternion as (w x y z)
  ROS defines quaternion as (x y z w)
  Therefore we have to swap some values
  Also we have to convert from NED to ENU frame,
  see the negative sign for y (q2) and z (q3)
     
    x                 z  x
   /                  | /
  /____y    ==>  y____|/
  |
  |z

  NED       ==>   ENU

  """

  if not map_frame_id:
    rospy.logwarn('Map frame id is not defined!')
    return

  if not baselink_translation_frame_id:
    rospy.logwarn('Baselink translation frame id is not defined!')
    return

  time = msg.header.stamp

  quat = (msg.q1, -msg.q2, -msg.q3, msg.q0)
  h = Header()
  h.stamp = time
  h.frame_id = baselink_translation_frame_id
  t_stamped = TransformStamped()
  t_stamped.header = h
  t_stamped.child_frame_id = estimated_baselink_frame_id
  t_stamped.transform = Transform()
  t_stamped.transform.translation = Vector3(0.0, 0.0, 0.0)
  t_stamped.transform.rotation = Quaternion(*quat)
  tf_broadcaster.sendTransform(t_stamped)

  pose_stamped = PoseStamped()
  pose_stamped.header = h
  pose_stamped.pose = Pose()
  pose_stamped.pose.position = Point(0, 0, 0)
  pose_stamped.pose.orientation = Quaternion(*quat)
  local_orient_pose_pub.publish(pose_stamped)

rospy.init_node(NODE_NAME)
rospy.Subscriber('attitude_quaternion', AttitudeQuaternion, attitude_quaternion_callback)
local_orient_pose_pub = rospy.Publisher('local_orientation_pose', PoseStamped, queue_size = 1)
local_orient_transform_pub = rospy.Publisher('local_orientation_transform', TransformStamped, queue_size = 1)
tf_broadcaster = tf2_ros.TransformBroadcaster()

baselink_translation_frame_id = rospy.get_param('~baselink_translation_frame_id')
estimated_baselink_frame_id   = rospy.get_param('~estimated_baselink_frame_id')

rospy.loginfo('%s: Assuming baselink translation (no orientation) frame id to be %s', NODE_NAME, baselink_translation_frame_id)
rospy.loginfo('%s: Assuming estimated baselink frame id to be %s', NODE_NAME, estimated_baselink_frame_id)

rospy.spin()
