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

class StampedGeoPoint(object):

  def __init__(self, geopoint, stamp):
    self.geopoint = geopoint
    self.stamp = stamp

  def __format__(self, fmt_spec):
    gp = [self.geopoint.latitude, self.geopoint.longitude, self.geopoint.z]
    gp = map(math.degrees, gp[:2]) + [gp[2]]
    return ("<StampedGeoPoint \n\tstamp: {} \n\tlat: {}"
           "\n\tlon: {}\n\talt: {}\n> ".format(self.stamp, *gp))

  def to_msg(self):
    h = Header()
    h.stamp = self.stamp
    h.frame_id = map_frame_id

    msg = NavSatFix()
    msg.header = h
    msg.latitude = math.degrees(self.geopoint.latitude)
    msg.longitude = math.degrees(self.geopoint.longitude)
    msg.altitude = self.geopoint.z
    return msg

current_stamped_geopoint = None
reference_stamped_geopoint = None

map_frame_id = None
baselink_translation_frame_id = None

def rtk_callback(msg):
  lat = msg.latitude
  lon = msg.longitude
  alt = msg.altitude

  if 0.0 in [lat, lon, alt]:
    rospy.logwarn_throttle(2, ('{}: Received a msg with lat, lon, alt'
                              ': 0, 0, 0'.format(NODE_NAME)))
    return

  wgs84 = nv.FrameE(name='WGS84')

  current_geopoint = wgs84.GeoPoint(
    latitude=lat,
    longitude=lon,
    z=alt,
    degrees=True)

  global current_stamped_geopoint
  current_stamped_geopoint = StampedGeoPoint(current_geopoint, msg.header.stamp)

  if reference_stamped_geopoint == None:
    rospy.logwarn_throttle(2, '{}: RTK Reference is not set!'.format(NODE_NAME))
    return

  '''get transformation from intial to current in Earth coordinate frame'''
  transform_in_E = nv.diff_positions(
    reference_stamped_geopoint.geopoint,
    current_stamped_geopoint.geopoint)

  '''get transformation in NED frame'''
  reference_frame = nv.FrameN(reference_stamped_geopoint.geopoint)
  transform_in_NED = transform_in_E.change_frame(reference_frame)
  transform_in_NED = transform_in_NED.pvector.ravel()

  '''get transformation in ENU frame (ROS compatible)'''
  transform_in_ENU = [
    transform_in_NED[1],
    transform_in_NED[0],
    transform_in_NED[2]
  ]

  h = Header()
  if ( current_stamped_geopoint.stamp == rospy.Time(0) ):
    h.stamp = rospy.get_rostime()
  else:
    h.stamp = current_stamped_geopoint.stamp
  h.frame_id = map_frame_id

  p_stamped = PointStamped()
  p_stamped.header = h
  p_stamped.point.x = transform_in_ENU[0]
  p_stamped.point.y = transform_in_ENU[1]
  p_stamped.point.z = transform_in_ENU[2]
  local_rtk_enu_position_pub.publish(p_stamped)

  t_stamped = TransformStamped()
  t_stamped.header = h
  t_stamped.child_frame_id = baselink_translation_frame_id
  t_stamped.transform = Transform()
  t_stamped.transform.translation = Vector3(*transform_in_ENU)
  t_stamped.transform.rotation = Quaternion(0, 0, 0, 1)
  tf_broadcaster.sendTransform(t_stamped)

def set_geo_reference(request):
  if current_stamped_geopoint == None:
    return TriggerResponse(False, 'Did not receive any RTK message so far!')

  # duration = (rospy.Time.now() - current_stamped_geopoint.stamp).to_sec()
  # if duration > 1:
  #   string_response = 'Last message was received {:.3f}s ago'.format(duration)
  #   rospy.logwarn('{}: {}'.format(NODE_NAME, string_response))
  #   return TriggerResponse(False, string_response)

  global reference_stamped_geopoint
  reference_stamped_geopoint = current_stamped_geopoint
  string_response = 'Reference was set to: {}'.format(reference_stamped_geopoint)
  rospy.loginfo('{}: {}'.format(NODE_NAME, string_response))
  return TriggerResponse(True, string_response)

rospy.init_node(NODE_NAME)

##
## TODO(asiron)
### fix autoset_geo_reference
### fix SDK to give correct timestamps with RTK data and uncomment the check above
 

autoset_geo_reference = rospy.get_param('~autoset_geo_reference')

map_frame_id = rospy.get_param('~map_frame_id')
baselink_translation_frame_id = rospy.get_param('~baselink_translation_frame_id')

rospy.loginfo('{}: Assuming map frame id to be: {}'.format(NODE_NAME, map_frame_id))
rospy.loginfo(('{}: Assuming baselink translation (no orientation) frame id'
               'to be: {}'.format(NODE_NAME, baselink_translation_frame_id)))

tf_broadcaster = tf2_ros.TransformBroadcaster()

rospy.Service('set_geo_reference', Trigger, set_geo_reference)
rospy.Subscriber('global_rtk_position', NavSatFix, rtk_callback)
local_rtk_enu_position_pub  = rospy.Publisher('local_rtk_enu_position',
                                              PointStamped,
                                              queue_size = 1)
reference_geopoint_pub = rospy.Publisher('rtk_reference_geopoint',
                                         NavSatFix,
                                         queue_size = 1,
                                         latch = True)

r = rospy.Rate(1)
while not rospy.is_shutdown():
  if reference_stamped_geopoint:
    reference_geopoint_pub.publish(reference_stamped_geopoint.to_msg())
  else:
    rospy.logwarn_throttle(1, ('{}: No RTK message received and no reference '
                              'was set!'.format(NODE_NAME)))
  try:
    r.sleep()
  except rospy.exceptions.ROSTimeMovedBackwardsException:
    pass
  except rospy.exceptions.ROSInterruptException:
    break
