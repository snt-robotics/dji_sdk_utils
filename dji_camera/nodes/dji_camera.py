#!/usr/bin/env python
import os
import sys
import yaml
import time
import csv

import numpy as np

import rospy
import tf2_ros

import threading, Queue

from functools import wraps, partial

from robot_geometry_msgs.msg import RobotState
from dji_sdk.srv import CameraAction

from std_srvs.srv import Trigger, TriggerResponse

class TakePictureThread(threading.Thread):

  def __init__(self, queue, id):
    super(TakePictureThread, self).__init__()
    self.queue = queue
    self.id = id
    rospy.wait_for_service('camera_action')
    self.camera_client = rospy.ServiceProxy('camera_action', CameraAction)

  def run(self):
    while not rospy.is_shutdown():
      try:
        msg = self.queue.get(block=False)
        rospy.loginfo("{}: Thread {:02d}: Taking msg from the queue...".format(node_name, self.id))
        self.handle_msg(msg)
      except Queue.Empty:
        pass
      rospy.sleep(0.01)

  def handle_msg(self, msg):

    global stopped
    if stopped:
      rospy.logwarn("{}: Camera is stopped!".format(node_name))
      return

    waypoint_reached_time = msg.header.stamp
    before_calling_camera = rospy.Time.now()


    try:
      self.camera_client(0)
    except rospy.ServiceException as exc:
      rospy.loginfo("{}: Thread {:02d}: Service did not process request: ".format(node_name, self.id, exc))


    rospy.loginfo("{}: Picture was taken!".format(node_name))

    global counter
    for pair in frame_id_pairs:

      source_frame_id = pair['source_frame_id']
      target_frame_id = pair['target_frame_id']
      writer = pair['writer']

      rospy.loginfo(('{}: Looking for transform '
                     'from {} to {} at time {:.3f}'
                     ).format(node_name,
                              source_frame_id,
                              target_frame_id,
                              waypoint_reached_time.to_sec()))


      waypoint_time = True
      transform = get_transform(source_frame_id, target_frame_id, waypoint_reached_time)
      if transform is None:
        rospy.logwarn(("{}: Obtaining transform at waypoint time failed!"
                       " Trying at camera picture taken time!").format(node_name))
        transform = get_transform(source_frame_id, target_frame_id, before_calling_camera)
        waypoint_time = False

      row = [
        counter,
        waypoint_reached_time.to_sec(),
        before_calling_camera.to_sec()
      ] + transform_msg_to_list(transform) + [waypoint_time]
      writer.writerow(dict(zip(fieldnames, row)))
      pair['file'].flush()

    counter += 1


def get_transform(source_frame, target_frame, at_time):
  transform = None
  try:
    transform = tf_buffer.lookup_transform(source_frame,
                                           target_frame,
                                           at_time,
                                           timeout=rospy.Duration(0.1))

  except (tf2_ros.LookupException,
          tf2_ros.ConnectivityException,
          tf2_ros.ExtrapolationException) as e:
    rospy.logwarn(e)
  return transform

def transform_msg_to_list(msg):
  if msg is None:
    return list(np.full((7,), np.nan))

  tran = msg.transform.translation
  rot  = msg.transform.rotation
  return [tran.x, tran.y, tran.z, rot.x, rot.y, rot.z, rot.w]

def start_camera(request):
  global stopped
  stopped = False
  return TriggerResponse(True, "Camera started!")

def stop_camera(request):
  global stopped
  stopped = True
  return TriggerResponse(True, "Camera stopped!")

def waypoint_reached_callback(msg, queue):
  try:
    queue.put(msg, block=False)
  except Queue.Full:
    pass

rospy.init_node('dji_camera')

stopped = True 

node_name = rospy.get_name()

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
rospy.loginfo('{}: Output directory is: {}'.format(node_name, output_directory))

if not os.path.exists(output_directory):
  os.makedirs(output_directory)

fieldnames = [
  'counter',
  'waypoint_stamp',
  'before_pic_stamp',
  'tx', 'ty', 'tz',
  'qx', 'qy', 'qz', 'qw',
  'waypoint_time__or__picture_taken_time'
]
rospy.loginfo(('{}: Loaded the following frame id pairs:'.format(node_name)))
for pair in frame_id_pairs:
  source_frame_id = pair['source_frame_id']
  target_frame_id = pair['target_frame_id']
  rospy.loginfo(('{}: Frame id pair is {} -> {}'
                ).format(node_name, source_frame_id, target_frame_id))

  filename = '{}__to__{}.csv'.format(
    source_frame_id.replace('/', '-'),
    target_frame_id.replace('/', '-')
  )
  filepath = os.path.join(output_directory, filename)

  f = open(filepath, 'w')
  writer = csv.DictWriter(f, fieldnames=fieldnames)
  writer.writeheader()

  pair['file'] = f
  pair['writer'] = writer

counter = 0

rospy.Service('~start', Trigger, start_camera)
rospy.Service('~stop', Trigger, stop_camera)

msg_queue = Queue.Queue()
callback = lambda msg: waypoint_reached_callback(msg, msg_queue)
rospy.Subscriber('waypoint_reached', RobotState, callback)

take_picture_threads = [TakePictureThread(msg_queue, id) for id in range(10)]
for t in take_picture_threads:
  t.start()

rospy.spin()

for pair in frame_id_pairs:
  pair['file'].close()

for t in take_picture_threads:
  t.join()
