#!/usr/bin/env python
"""
Project /iris/scan to PointCloud2 in link, then TF to world for EGO grid_map.
EGO-Planner grid_map uses point coordinates as world-frame positions
(no TF in cloudCallback); RViz still works with link-frame clouds via TF, but
occupancy must receive world-frame points.
"""
import copy
import rospy

import laser_geometry.laser_geometry as lg
import tf2_ros
import tf2_sensor_msgs
from sensor_msgs.msg import LaserScan, PointCloud2

TARGET_FRAME = "world"
SOURCE_FRAME = "link"

def scan_callback(msg, args):
    lp, tf_buffer, pub = args
    scan_filtered = copy.deepcopy(msg)
    max_range = msg.range_max

    range_list = list(scan_filtered.ranges)
    for i in range(len(range_list)):
        r = range_list[i]
        if r == float('inf') or r != r or r > max_range:
            range_list[i] = float('nan')
    scan_filtered.ranges = range_list

    pc2_link = lp.projectLaser(scan_filtered)
    pc2_link.header.frame_id = SOURCE_FRAME
    stamp = msg.header.stamp
    if msg.header.stamp.to_sec() > 0.0:
        stamp = msg.header.stamp
    else:
        stamp = rospy.Time.now()
    pc2_link.header.stamp = stamp

    try:
        transform = tf_buffer.lookup_transform(
            TARGET_FRAME,
            SOURCE_FRAME,
            stamp,
            rospy.Duration(0.2)
        )
        pc2_world = tf2_sensor_msgs.do_transform_cloud(pc2_link, transform)
        pc2_world.header.frame_id = TARGET_FRAME
        pc2_world.header.stamp = stamp
        pub.publish(pc2_world)
    except (
        tf2_ros.LookupException,
        tf2_ros.ConnectivityException,
        tf2_ros.ExtrapolationException,
    ) as err:
        rospy.logwarn_throttle(
            1.0,
            "scan2pointcloud: TF %s -> %s failed: %s",
            SOURCE_FRAME,
            TARGET_FRAME,
            err,
        )

if __name__ == '__main__':
    rospy.init_node('scan2pointcloud')
    lp = lg.LaserProjection()
    tf_buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buffer)
    pub = rospy.Publisher('/iris/points', PointCloud2, queue_size=10)
    rospy.Subscriber(
        '/iris/scan',
        LaserScan,
        scan_callback,
        callback_args=(lp, tf_buffer, pub),
        queue_size=10
    )
    rospy.loginfo(
        "scan2pointcloud: /iris/scan -> /iris/points (%s via TF from %s)",
        TARGET_FRAME,
        SOURCE_FRAME
    )
    rospy.spin()
