#!/usr/bin/env python
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

def attitude_quaternion_callback(data):
  '''
  DJI SDK defines quaternion as (w x y z)
  ROS defines quaternion as (x y z w)
  Therefore we have to swap some values
  '''
  quat = (data.q1, data.q2, data.q3, data.q0)
  header = data.header
  header.frame_id = 'm600/rtk_pos'
  t_stamped = TransformStamped()
  t_stamped.header = header
  t_stamped.child_frame_id  = 'm600/orientation'
  t_stamped.transform = Transform()
  t_stamped.transform.translation = Vector3(1.0, 0.0, 0.0)
  t_stamped.transform.rotation = Quaternion(*quat)
  tf_broadcaster.sendTransform(t_stamped)

  pose_stamped = PoseStamped()
  pose_stamped.header = header
  pose_stamped.pose = Pose()
  pose_stamped.pose.position = Point(0, 0, 0)
  pose_stamped.pose.orientation = Quaternion(*quat)
  local_orient_pose_pub.publish(pose_stamped)

local_orient_pose_pub = rospy.Publisher('local_orientation_pose', PoseStamped, queue_size = 1)
local_orient_transform_pub = rospy.Publisher('local_orientation_transform', TransformStamped, queue_size = 1)
tf_broadcaster = tf2_ros.TransformBroadcaster()

rospy.init_node('att_quat_to_transform')
rospy.Subscriber('attitude_quaternion', AttitudeQuaternion, attitude_quaternion_callback)
rospy.spin()
