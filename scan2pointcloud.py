#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan, PointCloud2
import laser_geometry.laser_geometry as lg
import copy

def scan_callback(msg):
    scan_filtered = copy.deepcopy(msg)
    max_range = msg.range_max

    range_list = list(scan_filtered.ranges)
    for i in range(len(range_list)):
        r = range_list[i]
        if r == float('inf') or r != r or r > max_range:
            range_list[i] = float('nan')
    scan_filtered.ranges = range_list

    lp = lg.LaserProjection()
    pc2_msg = lp.projectLaser(scan_filtered)
    pc2_msg.header.frame_id = "link"
    pub.publish(pc2_msg)

if __name__ == '__main__':
    rospy.init_node('scan2pointcloud')
    lp = lg.LaserProjection()
    sub = rospy.Subscriber('/iris/scan', LaserScan, scan_callback)
    pub = rospy.Publisher('/iris/points', PointCloud2, queue_size=10)
    rospy.loginfo("scan2pointcloud node start: /iris/scan -> /iris/points , frame_id = link")
    rospy.spin()
