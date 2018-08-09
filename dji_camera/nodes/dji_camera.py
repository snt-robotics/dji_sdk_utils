#!/usr/bin/env python
import os
import sys
import yaml
import time
import csv

import numpy as np

import rospy
import tf2_ros

from robot_geometry_msgs.msg import RobotState
from dji_sdk.srv import CameraAction

NODE_NAME = 'dji_camera'

def transform_msg_to_list(msg):
  if msg is None:
    return list(np.full((7,), np.nan))

  tran = msg.transform.translation
  rot  = msg.transform.rotation
  return [tran.x, tran.y, tran.z, rot.x, rot.y, rot.z, rot.w]

def waypoint_reached_callback(msg):
  waypoint_reached_time = msg.header.stamp
  before_calling_camera = rospy.Time.now()
  camera_client(0);

  for pair in frame_id_pairs:

    source_frame_id = pair['source_frame_id']
    target_frame_id = pair['target_frame_id']
    writer = pair['writer']

    rospy.loginfo(('{}: Looking for transform '
                   'from {} to {} at time {:.3f}'
                   ).format(NODE_NAME,
                            source_frame_id,
                            target_frame_id,
                            waypoint_reached_time.to_sec()))

    transform = None
    try:
      transform = tf_buffer.lookup_transform(source_frame_id,
                                             target_frame_id,
                                             waypoint_reached_time)

    except (tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException) as e:
      rospy.logwarn(e)

    row = [
      counter,
      waypoint_reached_time.to_sec(),
      before_calling_camera.to_sec()
    ] + transform_msg_to_list(transform)
    writer.writerow(dict(zip(fieldnames, row)))


  global counter
  counter += 1


rospy.init_node(NODE_NAME)

tf_buffer = tf2_ros.Buffer()
tf_listener = tf2_ros.TransformListener(tf_buffer)

frame_id_pairs = None
frame_id_pairs_file = rospy.get_param("~frame_id_pairs")
with open(frame_id_pairs_file, 'r') as stream:
  try:
    frame_id_pairs = yaml.load(stream)
  except yaml.YAMLError as exc:
    rospy.loginfo(e)
    sys.exit(os.EX_DATAERR)

assert len(frame_id_pairs) > 0

output_directory = rospy.get_param("~output_directory")
output_directory = os.path.join(output_directory, time.strftime('%Y%m%d-%H%M%S'))
rospy.loginfo('{}: Output directory is: {}'.format(NODE_NAME, output_directory))

if not os.path.exists(output_directory):
  os.makedirs(output_directory)

fieldnames = [
  'counter',
  'waypoint_stamp',
  'before_pic_stamp',
  'tx', 'ty', 'tz',
  'qx', 'qy', 'qz', 'qw'
]
rospy.loginfo(('{}: Loaded the following frame id pairs:'.format(NODE_NAME)))
for pair in frame_id_pairs:
  source_frame_id = pair['source_frame_id']
  target_frame_id = pair['target_frame_id']
  rospy.loginfo(('{}: Frame id pair is {} -> {}'
                ).format(NODE_NAME, source_frame_id, target_frame_id))

  filename = '{}-to-{}.csv'.format(source_frame_id, target_frame_id)
  filepath = os.path.join(output_directory, filename)

  f = open(filepath, 'w')
  writer = csv.DictWriter(f, fieldnames=fieldnames)
  writer.writeheader()

  pair['file'] = f
  pair['writer'] = writer

counter = 0

rospy.wait_for_service('camera_action')
camera_client = rospy.ServiceProxy('camera_action', CameraAction)

rospy.Subscriber('waypoint_reached', RobotState, waypoint_reached_callback)

rospy.spin()

for pair in frame_id_pairs:
  pair['file'].close()
