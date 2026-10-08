#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
══════════════════════════════════════════════════════════════════════════════
ROS CAMERA PUBLISHER NODE CHO OAK-D S2 & VPU SPATIAL AI (CAMERA_STREAMER_NODE.PY)
Package: jetbot_slam
Topics:
  - /stereo_inertial_publisher/color/image (sensor_msgs/Image, bgr8)
  - /stereo_inertial_publisher/stereo/depth (sensor_msgs/Image, 16UC1 mm)
  - /obstacle_distance (std_msgs/Float32)
  - /spatial_objects (std_msgs/String - JSON)
══════════════════════════════════════════════════════════════════════════════
"""

import os
import sys

# Đảm bảo import được các module từ thư mục gốc của package jetbot_slam
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PACKAGE_ROOT = os.path.dirname(SCRIPT_DIR)
if PACKAGE_ROOT not in sys.path:
    sys.path.insert(0, PACKAGE_ROOT)

from modules.camera_streamer import run_ros_camera_node

if __name__ == '__main__':
    run_ros_camera_node()
