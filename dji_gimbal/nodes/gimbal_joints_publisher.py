#!/usr/bin/env python
"""
Subscribes to Gimbal messages and republishes it as JointState message.
So that we can transform to the Camera frame.

Gimbal's yaw value is given with respect to North not drone's base_link.
Therefore, we correct for it by subtracting the yaw to north value
"""

import rospy, math, tf_conversions
from operator import sub
from dji_sdk.msg import Gimbal, AttitudeQuaternion
from std_msgs.msg import Header
from sensor_msgs.msg import JointState

NODE_NAME = 'gimbal_joints_publisher'
JOINTS = ['yaw', 'roll', 'pitch']

baselink_frame_id = None
subtract_orientation = None

current_abs_orientation = None
convert_degrees_to_std_range = lambda x: ( x + 180) % 360 - 180

def attitude_quaternion_callback(msg):
  q = (msg.q1, msg.q2, msg.q3, msg.q0)
  r, p, y = tf_conversions.transformations.euler_from_quaternion(q)
  angles = map(convert_degrees_to_std_range, [y, r, p])
  global current_abs_orientation
  current_abs_orientation = angles

def gimbal_callback(msg):

  h = Header()
  h.stamp = msg.header.stamp
  h.frame_id = baselink_frame_id

  if not current_abs_orientation and subtract_orientation:
    rospy.logwarn('%s: Did NOT receive ~attitude_quaternion msg, so can\'t corrent to north', NODE_NAME)
    return

  gimbal_positions = [msg.yaw, msg.roll, msg.pitch]
  if subtract_orientation:
    gimbal_positions = map(sub, gimbal_positions, current_abs_orientation)
  gimbal_positions = map(math.radians, gimbal_positions)

  joint_states = JointState()
  joint_states.header = h
  joint_states.name = JOINTS
  joint_states.position = gimbal_positions
  joint_states.velocity  = [0, 0, 0]
  joint_states.effort    = [0, 0, 0]
  gimbal_joint_states_pub.publish(joint_states)

rospy.init_node(NODE_NAME)

baselink_frame_id = rospy.get_param('~baselink_frame_id')
subtract_orientation = rospy.get_param('~subtract_orientation')

rospy.Subscriber('gimbal', Gimbal, gimbal_callback)

if subtract_orientation: 
  rospy.Subscriber('attitude_quaternion', AttitudeQuaternion, attitude_quaternion_callback)
  rospy.loginfo('Gimbal Yaw will be corrected to base_link by subtracting the angle to North')

gimbal_joint_states_pub = rospy.Publisher('gimbal_joint_states', JointState, queue_size = 1)

rospy.spin()
