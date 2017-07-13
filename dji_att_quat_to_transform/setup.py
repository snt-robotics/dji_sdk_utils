## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
  packages=['dji_att_quat_to_transform'],
  package_dir={'': 'src'},
  requires=['std_msgs', 'rospy', 'geometry_msgs', 'tf', 'tf2_ros' 'dji_sdk']
)

setup(**setup_args)