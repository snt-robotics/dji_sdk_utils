#!/usr/bin/env python
import rospy, math
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

current_position = None
initial_position = None
last_position_timestamp = None

map_frame_id = None
baselink_translation_frame_id = None

def warn_every(steps = 50):  
  class nonlocal:
    counter = 0

  def logwarn(msg):
    if nonlocal.counter % steps == 0:
      rospy.logwarn('%s: %s', NODE_NAME, msg)
    nonlocal.counter += 1
  return logwarn

def geopoint_to_list(gp, degrees=True):
  pos = [gp.latitude, gp.longitude, gp.z]
  return pos if not degrees else map(math.degrees, pos[:2]) + [pos[2]]

def a3_rtk_callback(msg):

  if not map_frame_id:
    rospy.logwarn('%s: ~map_frame_id is not defined!', NODE_NAME)
    return

  if not baselink_translation_frame_id:
    rospy.logwarn('%s: ~baselink_translation_frame_id is not defined!', NODE_NAME)
    return

  lat = msg.latitude_RTK
  lon = msg.longitude_RTK
  alt = msg.height_above_sea_RTK

  if 0.0 in [lat, lon, alt]:
    warn_invalid_position('Received a msg with lat, lon, alt: 0, 0, 0')
    return
    
  wgs84 = nv.FrameE(name='WGS84')

  global current_position, last_position_timestamp
  current_position = wgs84.GeoPoint(latitude=lat, longitude=lon, z=alt, degrees=True)
  last_position_timestamp = rospy.Time.now()

  if initial_position == None:
    warn_no_initial_position('Initial position is not set!')
    return

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
    -transform_in_NED[2]
  ]

  h = Header()
  h.stamp = rospy.Time.now()
  h.frame_id = map_frame_id

  p_stamped = PoseStamped()
  p_stamped.header = h
  p_stamped.pose = Pose()
  p_stamped.pose.position = Point(*transform_in_ENU)
  p_stamped.pose.orientation = Quaternion(0, 0, 0, 1)
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
    return TriggerResponse(False, 'Did not receive any A3_RTK message so far!')
  
  last_message_dur = (rospy.Time.now() - last_position_timestamp).to_sec()
  if last_message_dur > 2:
    string_response = 'Last message was received {:.3f}s ago'.format(last_message_dur)
    rospy.logwarn('%s: %s', NODE_NAME, string_response)
    return TriggerResponse(False, string_response)

  global initial_position
  initial_position = current_position
  position_degrees = geopoint_to_list(initial_position, degrees=True)
  string_response = 'Initial position set to: {}, {}, {}'.format(*position_degrees)
  rospy.loginfo('%s: %s', NODE_NAME, string_response)
  return TriggerResponse(True, string_response)

warn_invalid_position = warn_every(50)
warn_no_initial_position = warn_every(50)

rospy.init_node(NODE_NAME)
rospy.Subscriber('A3_RTK', A3RTK, a3_rtk_callback)
rospy.Service('set_current_position_as_initial', Trigger, set_current_position_as_initial)
tf_broadcaster = tf2_ros.TransformBroadcaster()
global_position_pub = rospy.Publisher('global_rtk', PoseStamped, queue_size = 1)

map_frame_id = rospy.get_param('~map_frame_id')
baselink_translation_frame_id = rospy.get_param('~baselink_translation_frame_id')

rospy.loginfo('%s: Assuming map frame id to be: %s', NODE_NAME, map_frame_id)
rospy.loginfo('%s: Assuming baselink translation (no orientation) frame id to be: %s', NODE_NAME, baselink_translation_frame_id)

rospy.spin()
