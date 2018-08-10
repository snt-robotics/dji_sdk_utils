#!/usr/bin/env python
"""
Subscribes to Gimbal messages and republishes it as JointState message.
So that we can transform to the Camera frame.

!!!!!!
!!!!!!
Gimbal's values are given with respect to an NED frame and not the UAV's base_link.
Therefore, we correct for it by subtracting the current absolute position in NED frame
from the gimbal angles
"""

import rospy, math, tf_conversions, sys
from operator import sub

from geometry_msgs.msg import Vector3Stamped, QuaternionStamped
from std_msgs.msg import Header
from sensor_msgs.msg import JointState

NODE_NAME = 'gimbal_joints_publisher'
JOINTS = ['yaw', 'roll', 'pitch']

baselink_frame_id = None
gimbal_ref_frame = None

current_abs_orientation = None
convert_radians_to_std_range = lambda x: ( x + math.pi ) % (2 * math.pi) - math.pi

def attitude_quaternion_callback(msg):
  q = (msg.quaternion.x, msg.quaternion.y, msg.quaternion.z, msg.quaternion.w)
  r, p, y = tf_conversions.transformations.euler_from_quaternion(q)
  angles = map(convert_radians_to_std_range, [y, r, p])
  global current_abs_orientation
  current_abs_orientation = angles

def gimbal_callback(msg):

  h = Header()
  h.stamp = msg.header.stamp
  h.frame_id = baselink_frame_id

  if not current_abs_orientation:
    rospy.logwarn_throttle(1, '{}: Did NOT receive ~attitude_quaternion msg, so can\'t corrent to north'.format(NODE_NAME))
    return

  gimbal_positions = map(math.radians, [msg.vector.z, msg.vector.y, msg.vector.x])
  if gimbal_ref_frame == 'ned':
    gimbal_positions = map(sub, gimbal_positions, current_abs_orientation)
  elif gimbal_ref_frame == 'ned_without_yaw':
    r, p = map(sub, gimbal_positions[1:], current_abs_orientation[1:])
    gimbal_positions = [gimbal_positions[0], r, p]

  joint_states = JointState()
  joint_states.header = h
  joint_states.name = JOINTS
  joint_states.position = gimbal_positions
  joint_states.velocity  = [0, 0, 0]
  joint_states.effort    = [0, 0, 0]
  gimbal_joint_states_pub.publish(joint_states)

rospy.init_node(NODE_NAME)

baselink_frame_id = rospy.get_param('~baselink_frame_id')
gimbal_ref_frame = rospy.get_param('~gimbal_ref_frame')

rospy.Subscriber('gimbal_angle', Vector3Stamped, gimbal_callback)

if gimbal_ref_frame == 'ned':
  rospy.loginfo('Gimbal Yaw will be corrected to base_link by subtracting the angle to North')
elif gimbal_ref_frame == 'ned_without_yaw':
  rospy.loginfo("Gimbal's angles will be corrected - excluding the yaw angle (Ronin-MX")
else:
  rospy.logfatal("Unrecognized param value for ~gimbal_ref_frame")
  sys.exit()

rospy.Subscriber('attitude', QuaternionStamped, attitude_quaternion_callback)
gimbal_joint_states_pub = rospy.Publisher('gimbal_joint_states', JointState, queue_size = 1)

rospy.spin()
