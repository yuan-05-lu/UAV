#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""兼容入口：请使用 ego_metrics_logger.py（Gazebo / rosbag 同款指标）。"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ego_metrics_logger import EgoMetricsLogger
import rospy

if __name__ == "__main__":
    try:
        EgoMetricsLogger()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass