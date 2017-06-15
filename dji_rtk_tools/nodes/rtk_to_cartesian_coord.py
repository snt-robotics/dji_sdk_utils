#!/usr/bin/env python
import rospy
import tf, tf2_ros
import nvector as nv
from dji_sdk.msg import A3RTK
from std_msgs.msg import Header
from std_srvs.srv import Trigger, TriggerResponse
from geometry_msgs.msg import (
  TransformStamped,
  Transform,
  PoseStamped,
  Pose,
  Quaternion, 
  Vector3,
  Point
)

NODE_NAME = 'rtk_to_cartesian_coord'

last_position_ECEF = None
initial_position_ECEF = None

def a3_rtk_callback(msg):
  lat = msg.latitude_RTK
  lon = msg.longitude_RTK
  alt = msg.height_above_sea_RTK

  wgs84 = nv.FrameE(name='WGS84')

  current_pos = wgs84.GeoPoint(latitude=lat, longitude=lon, z=alt, degrees=True)
  current_pos_ECEF = current_pos.to_ecef_vector().pvector.ravel()

  global last_position_ECEF
  last_position_ECEF = current_pos_ECEF

  if initial_position_ECEF == None:
    rospy.logwarn('Initial position is not set!')
    return

  postion_in_NED_frame = current_pos_ECEF - initial_position_ECEF
  

  h = Header()
  h.stamp = rospy.Time.now()
  h.frame_id = world_frame_id

  p_stamped = PoseStamped()
  p_stamped.header = h
  p_stamped.pose = Pose()
  p_stamped.pose.position = Point(*postion_in_NED_frame)
  p_stamped.pose.orientation = Quaternion(0, 0, 0, 1)
  global_position_pub.publish(p_stamped)

  t_stamped = TransformStamped()
  t_stamped.header = h
  t_stamped.child_frame_id = baselink_translation_frame_id
  t_stamped.transform = Transform()
  t_stamped.transform.translation = Vector3(*postion_in_NED_frame)
  t_stamped.transform.rotation = Quaternion(0, 0, 0, 1)
  tf_broadcaster.sendTransform(t_stamped)

def set_current_position_as_initial(request):
  if last_position_ECEF == None:
    return TriggerResponse(False, 'Did not receive any A3_RTK message so far!')
  global initial_position_ECEF
  initial_position_ECEF = last_position_ECEF
  string_response = 'Initial position set to: {} in ECEF frame'.format(initial_position_ECEF)
  return TriggerResponse(True, string_response)


rospy.init_node(NODE_NAME)
rospy.Subscriber('A3_RTK', A3RTK, a3_rtk_callback)
rospy.Service('set_current_position_as_initial', Trigger, set_current_position_as_initial)
tf_broadcaster = tf2_ros.TransformBroadcaster()
global_position_pub = rospy.Publisher('global_rtk', PoseStamped, queue_size = 1)

world_frame_id = rospy.get_param('~world_frame_id')
baselink_translation_frame_id = rospy.get_param('~baselink_translation_frame_id')

rospy.loginfo('%s: Assuming world frame id to be: %s', NODE_NAME, world_frame_id)
rospy.loginfo('%s: Assuming baselink translation (no orientation) frame id to be: %s', NODE_NAME, baselink_translation_frame_id)

rospy.spin()
