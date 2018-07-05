#!/usr/bin/env python
import rospy, math, collections
import tf, tf2_ros
import nvector as nv
import numpy as np

from functools import wraps, partial

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

WGS84 = nv.FrameE(name='WGS84')

""" TODO(asiron) convert into parameter """
HISTORY_LENGTH = 30
EXPECTED_FREQ = 5.0
LAST_MEASUREMENT_DELAY_THRESHOLD = 0.5
MEASUREMENT_UPDATE_FREQUENCY_TOLERANCE = 1.0

LAT_VAR_TH = 1e-10
LON_VAR_TH = 1e-10
ALT_VAR_TH = 1e-3

def logged_service_callback(f, throttled = True):

  if throttled:
    loginfo = partial(rospy.loginfo_throttle, 1)
    logwarn = partial(rospy.logwarn_throttle, 1)
  else:
    loginfo = rospy.loginfo
    logwarn = rospy.logwarn

  @wraps(f)
  def wrapper(*args, **kwds):
    ret_val = f(*args, **kwds)
    message = "{}: {}".format(NODE_NAME, ret_val.message)
    loginfo(message) if ret_val.success else logwarn(message)
    return ret_val

  return wrapper

class StampedGeoPoint(object):

  def __init__(self, geopoint, stamp):
    self.geopoint = geopoint

    """ TODO(asiron) remove when OSDK is fixed! """
    if stamp == rospy.Time(0):
      self.stamp = rospy.get_rostime()
    else:
      self.stamp = stamp

  def __format__(self, fmt_spec):
    gp = self.to_vec()
    return ("<StampedGeoPoint \n\tstamp: {} \n\tlat: {}"
           "\n\tlon: {}\n\talt: {}\n> ".format(self.stamp, *gp))

  def __eq__(self, other):
    if isinstance(other, StampedGeoPoint):
      return (self.geopoint.latitude == other.geopoint.latitude) \
        and (self.geopoint.longitude == other.geopoint.longitude) \
        and (self.geopoint.z == other.geopoint.z)
    return False

  def __ne__(self, other):
    if isinstance(other, StampedGeoPoint):
      return not (self == other)
    return False

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

  def to_vec(self, degrees=True):
    vec = [self.geopoint.latitude, self.geopoint.longitude, self.geopoint.z]
    return map(math.degrees, vec[:2]) + [vec[2]] if degrees else vec

measurement_history = collections.deque(maxlen=HISTORY_LENGTH)

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

  current_geopoint = WGS84.GeoPoint(
    latitude=lat,
    longitude=lon,
    z=alt,
    degrees=True)

  current_stamped_geopoint = StampedGeoPoint(current_geopoint, msg.header.stamp)

  if len(measurement_history) == 0:
    measurement_history.append(current_stamped_geopoint)
  else:
    last_measurement = measurement_history[-1]
    if last_measurement != current_stamped_geopoint:
      measurement_history.append(current_stamped_geopoint)
    else:
      return

  if reference_stamped_geopoint is None and autoset_geo_reference:
    set_geo_reference()

  if reference_stamped_geopoint is None:
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
  h.stamp = current_stamped_geopoint.stamp
  h.frame_id = map_frame_id

  p_stamped = PointStamped()
  p_stamped.header = h
  p_stamped.point.x = transform_in_ENU[0]
  p_stamped.point.y = transform_in_ENU[1]
  p_stamped.point.z = transform_in_ENU[2]
  local_rtk_enu_position_pub.publish(p_stamped)

  if(publish_tf):
    t_stamped = TransformStamped()
    t_stamped.header = h
    t_stamped.child_frame_id = baselink_translation_frame_id
    t_stamped.transform = Transform()
    t_stamped.transform.translation = Vector3(*transform_in_ENU)
    t_stamped.transform.rotation = Quaternion(0, 0, 0, 1)
    tf_broadcaster.sendTransform(t_stamped)

@logged_service_callback
def set_geo_reference(request=None):
  if len(measurement_history) == 0:
    return TriggerResponse(False, 'Did not receive any RTK message so far!')
  elif len(measurement_history) != HISTORY_LENGTH:
    return TriggerResponse(False, ('Did not receive all {} needed '
                                   'RTK messages!'.format(HISTORY_LENGTH)))

  last_measurement = measurement_history[-1]
  duration = (rospy.Time.now() - last_measurement.stamp).to_sec()
  if duration > LAST_MEASUREMENT_DELAY_THRESHOLD:
    string_response = 'Last message was received {:.3f}s ago'.format(duration)
    return TriggerResponse(False, string_response)

  oldest_measurement = measurement_history[0]
  total_duration = (last_measurement.stamp - oldest_measurement.stamp).to_sec()
  actual_freq = HISTORY_LENGTH / total_duration
  frequency_err = abs(EXPECTED_FREQ - actual_freq)
  if frequency_err > MEASUREMENT_UPDATE_FREQUENCY_TOLERANCE:
    string_response = ('Last {N} messages were received with a'
                       'freqency of {actual:.3f} instead of '
                       '{expected:.3f}'.format(N=HISTORY_LENGTH,
                                              actual=actual_freq,
                                              expected=EXPECTED_FREQ))
    return TriggerResponse(False, string_response)

  measurements = np.array(map(lambda m: m.to_vec(), measurement_history))
  measurement_covariance = np.cov(measurements, rowvar=False).diagonal()
  measurement_mean = np.mean(measurements, axis=0)

  lat_var, lon_var, alt_var = measurement_covariance
  if (lat_var > LAT_VAR_TH) or (lon_var > LON_VAR_TH) or (alt_var > ALT_VAR_TH):
    string_response = ('Calculated diagnoal covariance matrix '
                       '(lat, lon, alt) was {} and the limits '
                       'are {} {} {}'.format(measurement_covariance,
                                             LAT_VAR_TH,
                                             LON_VAR_TH,
                                             ALT_VAR_TH))
    return TriggerResponse(False, string_response)

  lat, lon, alt = measurement_mean
  reference_geopoint = WGS84.GeoPoint(
    latitude=lat,
    longitude=lon,
    z=alt,
    degrees=True)

  global reference_stamped_geopoint
  reference_stamped_geopoint = StampedGeoPoint(reference_geopoint,
                                             last_measurement.stamp)
  string_response = 'Reference was set to: {}'.format(reference_stamped_geopoint)
  return TriggerResponse(True, string_response)

rospy.init_node(NODE_NAME)

##
## TODO(asiron)
### fix SDK to give correct timestamps with RTK data and uncomment the check above

autoset_geo_reference = rospy.get_param('~autoset_geo_reference')
publish_tf = rospy.get_param('~publish_tf')

map_frame_id = rospy.get_param('~map_frame_id')
baselink_translation_frame_id = rospy.get_param('~baselink_translation_frame_id')

rospy.loginfo('{}: Assuming map frame id to be: {}'.format(NODE_NAME, map_frame_id))
rospy.loginfo(('{}: Assuming baselink translation (no orientation) frame id'
               'to be: {}'.format(NODE_NAME, baselink_translation_frame_id)))

if(publish_tf):
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
    rospy.logwarn_throttle(10, ('{}: No RTK message received and no reference '
                              'was set!'.format(NODE_NAME)))
  try:
    r.sleep()
  except rospy.exceptions.ROSTimeMovedBackwardsException:
    pass
  except rospy.exceptions.ROSInterruptException:
    break
