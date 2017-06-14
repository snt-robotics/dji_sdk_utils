#!/usr/bin/env python
import rosbag, argparse, simplekml, pytz
from datetime import datetime
from tqdm import tqdm

DEFAULT_RTK_TOPIC = "/dji_sdk/A3_RTK"
DEFAULT_MSG_OUTPUT_FREQ  = 1.0
DEFAULT_MSG_INPUT_FREQ = 50.0

if __name__ == '__main__':
  parser = argparse.ArgumentParser(description="ROS Bag RTK extraction and conversion to KML")

  parser.add_argument('output', type=str, help='Output filepath to a ROSBAG containing dji_sdk/A3RTK messages')
  parser.add_argument('-r', '--rosbag', type=str, help='ROSBAG file containing A3_RTK messages', required=True)
  parser.add_argument('-t', '--topic', type=str,  help='A3_RTK topic', default=DEFAULT_RTK_TOPIC)
  parser.add_argument('-f', '--freq', type=float,  help='Frequncy at which we sample messages', default=DEFAULT_MSG_OUTPUT_FREQ)
  args = parser.parse_args()

  bag = rosbag.Bag(args.rosbag)

  skip_n = int(DEFAULT_MSG_INPUT_FREQ / args.freq)
  coords, timestamps = [], []

  for n, (_, msg, t) in tqdm(enumerate(bag.read_messages(topics=[args.topic])), ascii=True, ncols=60):

    if n % skip_n:
      continue

    t = float('{}.{}'.format(t.secs, t.nsecs))
    t = datetime.fromtimestamp(t).isoformat()
    t = simplekml.TimeStamp(t)

    lat = msg.latitude_RTK
    lon = msg.longitude_RTK
    alt = msg.height_above_sea_RTK

    if (lon, lat, alt) == (0.0, 0.0, 0.0):
      continue

    coords += [(lon, lat, alt)]
    timestamps += [t]

  bag.close()

  kml = simplekml.Kml(name="Tracks", open=1)
  doc = kml.newdocument(name='GPS device', snippet=simplekml.Snippet('Created Wed Jun 2 15:33:39 2010'))
  doc.lookat.gxtimespan.begin = '2010-05-28T02:02:09Z'
  doc.lookat.gxtimespan.end = '2010-05-28T02:02:56Z'
  doc.lookat.longitude = -122.205544
  doc.lookat.latitude = 37.373386
  doc.lookat.range = 1300.000000

  fol = doc.newfolder(name='UAV Track')
  trk = fol.newgxtrack(name='2010-05-28T01:16:35.000Z')

  trk.newwhen(timestamps)
  trk.newgxcoord(coords)

  trk.stylemap.normalstyle.iconstyle.icon.href = 'http://earth.google.com/images/kml-icons/track-directional/track-0.png'
  trk.stylemap.normalstyle.linestyle.color = '99ffac59'
  trk.stylemap.normalstyle.linestyle.width = 6
  trk.stylemap.highlightstyle.iconstyle.icon.href = 'http://earth.google.com/images/kml-icons/track-directional/track-0.png'
  trk.stylemap.highlightstyle.iconstyle.scale = 1.2
  trk.stylemap.highlightstyle.linestyle.color = '99ffac59'
  trk.stylemap.highlightstyle.linestyle.width = 8


  kml.save(args.output)
