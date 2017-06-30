#!/usr/bin/env python
"""
Subscribes to Gimbal messages and republishes it as JointState message.
So that we can transform to the Camera frame.

Gimbal's yaw value is given with respect to North not drone's base_link.
Therefore, we correct for it by subtracting the yaw to north value
"""

import rospy, math, tf_conversions
from dji_sdk.msg import Gimbal, AttitudeQuaternion
from std_msgs.msg import Header
from sensor_msgs.msg import JointState

NODE_NAME = 'gimbal_joints_publisher'
JOINTS = ['yaw', 'roll', 'pitch']

baselink_frame_id = None

current_yaw_to_north = None
convert_degrees_to_std_range = lambda x: ( x + math.pi) % (2 * math.pi ) - math.pi

def attitude_quaternion_callback(msg):
  q = (msg.q1, msg.q2, msg.q3, msg.q0)
  _, _, yaw = tf_conversions.transformations.euler_from_quaternion(q)
  yaw = convert_degrees_to_std_range(yaw)
  yaw = math.degrees(yaw)
  global current_yaw_to_north
  current_yaw_to_north = yaw

def gimbal_callback(msg):

  if not baselink_frame_id:
    rospy.logwarn('%s: ~baselink_frame_id is not defined!', NODE_NAME)
    return

  h = Header()
  h.stamp = msg.header.stamp
  h.frame_id = baselink_frame_id

  if not current_yaw_to_north:
    rospy.logwarn('%s: Did NOT receive ~attitude_quaternion msg, so can\'t corrent to north', NODE_NAME)
    return

  joint_states = JointState()
  joint_states.header = h
  joint_states.name = JOINTS
  joint_states.position = map(math.radians, [msg.yaw - current_yaw_to_north, msg.roll, msg.pitch])
  joint_states.velocity  = [0, 0, 0]
  joint_states.effort    = [0, 0, 0]
  gimbal_joint_states_pub.publish(joint_states)

rospy.init_node(NODE_NAME)
rospy.Subscriber('gimbal', Gimbal, gimbal_callback)
rospy.Subscriber('attitude_quaternion', AttitudeQuaternion, attitude_quaternion_callback)

gimbal_joint_states_pub = rospy.Publisher('gimbal_joint_states', JointState, queue_size = 1)

baselink_frame_id = rospy.get_param('~baselink_frame_id')

rospy.spin()
