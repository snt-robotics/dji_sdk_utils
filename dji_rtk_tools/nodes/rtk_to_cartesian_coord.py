#!/usr/bin/env python
import rospy, math
import tf, tf2_ros
import nvector as nv
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Header
from std_srvs.srv import Trigger, TriggerResponse
from geometry_msgs.msg import (
  TransformStamped,
  Transform,
  PoseStamped,
  Pose,
  Quaternion,
  Vector3,
  Point,
  PointStamped
)

NODE_NAME = 'rtk_to_cartesian_coord'

current_position = None
initial_position = None
last_position_timestamp = None

map_frame_id = None
baselink_translation_frame_id = None

def geopoint_to_msg(gp):
  h = Header()
  h.stamp = rospy.Time.now()
  h.frame_id = map_frame_id

  msg = NavSatFix()
  msg.header = h
  msg.latitude = gp.latitude
  msg.longitude = gp.longitude
  msg.altitude = gp.z
  return msg

def geopoint_to_list(gp, degrees=True):
  pos = [gp.latitude, gp.longitude, gp.z]
  return pos if not degrees else map(math.degrees, pos[:2]) + [pos[2]]


def rtk_callback(msg):

  lat = msg.latitude
  lon = msg.longitude
  alt = msg.altitude

  if 0.0 in [lat, lon, alt]:
    rospy.logwarn_throttle(2, 'Received a msg with lat, lon, alt: 0, 0, 0')
    return

  wgs84 = nv.FrameE(name='WGS84')

  global current_position, last_position_timestamp
  current_position = wgs84.GeoPoint(latitude=lat, longitude=lon, z=alt, degrees=True)
  last_position_timestamp = rospy.Time.now()

  if initial_position == None:
    rospy.logwarn_throttle(2, 'Initial position is not set!')
    return

  calc_earth_sphere(current_position, initial_position, last_position_timestamp)


  '''get transformation from intial to current in Earth coordinate frame'''
  transform_in_E = nv.diff_positions(initial_position, current_position)

  '''get transformation in NED frame'''
  intial_frame = nv.FrameN(initial_position)
  transform_in_NED = transform_in_E.change_frame(intial_frame)
  transform_in_NED = transform_in_NED.pvector.ravel()

  '''get transformation in ENU frame (ROS compatible)'''
  transform_in_ENU = [
    transform_in_NED[1],
    transform_in_NED[0],
    transform_in_NED[2]
  ]

  h = Header()
  h.stamp = last_position_timestamp
  h.frame_id = map_frame_id

  p_stamped = PointStamped()
  p_stamped.header = h
  p_stamped.point.x = transform_in_ENU[0]
  p_stamped.point.y = transform_in_ENU[1]
  p_stamped.point.z = transform_in_ENU[2]
  global_position_pub.publish(p_stamped)

  t_stamped = TransformStamped()
  t_stamped.header = h
  t_stamped.child_frame_id = baselink_translation_frame_id
  t_stamped.transform = Transform()
  t_stamped.transform.translation = Vector3(*transform_in_ENU)
  t_stamped.transform.rotation = Quaternion(0, 0, 0, 1)
  tf_broadcaster.sendTransform(t_stamped)

def set_current_position_as_initial(request):
  if current_position == None:
    return TriggerResponse(False, 'Did not receive any RTK message so far!')

  last_message_dur = (rospy.Time.now() - last_position_timestamp).to_sec()
  if last_message_dur > 2:
    string_response = 'Last message was received {:.3f}s ago'.format(last_message_dur)
    rospy.logwarn('%s: %s', NODE_NAME, string_response)
    return TriggerResponse(False, string_response)

  global initial_position
  initial_position = current_position
  string_response = 'Initial position set to: {}, {}, {}'\
    .format(*geopoint_to_list(initial_position, degrees=True))
  rospy.loginfo('%s: %s', NODE_NAME, string_response)
  return TriggerResponse(True, string_response)

rospy.init_node(NODE_NAME)

map_frame_id = rospy.get_param('~map_frame_id')
baselink_translation_frame_id = rospy.get_param('~baselink_translation_frame_id')

rospy.loginfo('%s: Assuming map frame id to be: %s', NODE_NAME, map_frame_id)
rospy.loginfo('%s: Assuming baselink translation (no orientation) frame id to be: %s', \
  NODE_NAME, baselink_translation_frame_id)

tf_broadcaster = tf2_ros.TransformBroadcaster()

rospy.Subscriber('dji_sdk/rtk_position', NavSatFix, rtk_callback)

rospy.Service('set_current_position_as_initial', Trigger, set_current_position_as_initial)
global_position_pub  = rospy.Publisher('rtk_enu', PointStamped, queue_size = 1)
initial_position_pub = rospy.Publisher('rtk_initial_position', NavSatFix, queue_size = 1)

r = rospy.Rate(1)
while not rospy.is_shutdown():
  if initial_position:
    msg = geopoint_to_msg(initial_position)
    initial_position_pub.publish(msg)
  try:
    r.sleep()
  except rospy.exceptions.ROSTimeMovedBackwardsException:
    pass
  except rospy.exceptions.ROSInterruptException:
    break
