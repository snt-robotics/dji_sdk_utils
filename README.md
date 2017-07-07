# dji_sdk_utils
Copyright (C) 2017 - Maciej Żurad, University of Luxembourg

Collection of packages and tools for using DJI UAVS in ROS

### List of packages

1. dji_att_quat_to_transform
  * Publishes `AttitudeQuaternion` messages as a TF transform and geometry_msgs
2. dji_gimbal
  * Publishes gimbal's state with `robot_state_publisher`
  * Supported gimbals (URDF models):
    * DJI Zenmuse X3
    * DJI Ronin-MX (tunning required as different cameras are mounted differently)
3. dji_rtk_tools
  * rtk_to_cartesian_coord.py
    * `/dji_sdk/A3_RTK` messages into local Euclidean frame (NED) given initial position
  * rtk_rosbag_to_kml.py
    * Converts ROSBAG with `/dji_sdk/A3_RTK` messages into a Google's KML file, so that
      it can be loaded into Google Earth. However, accuracy of Google Maps is most of the time
      lower than D-RTK's precision.
4. dji_launchers
  * Collection of launcher scripts for DJI M100 and M600 UAVs. The M100 launcher is prepared for a flight arena at SnT, University of Luxembourg.

### Python dependecies

```
pip install simplekml tqdm nvector
sudo apt-get install "ros-${ROS_DISTRO}-tf-conversions"
sudo apt-get install "ros-${ROS_DISTRO}-tf2*"
```