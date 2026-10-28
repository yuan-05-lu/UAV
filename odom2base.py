#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import tf
from nav_msgs.msg import Odometry

br = tf.TransformBroadcaster()

def callback(msg):

    px = msg.pose.pose.position.x
    py = msg.pose.pose.position.y
    pz = msg.pose.pose.position.z
    qx = msg.pose.pose.orientation.x
    qy = msg.pose.pose.orientation.y
    qz = msg.pose.pose.orientation.z
    qw = msg.pose.pose.orientation.w

    br.sendTransform(
        (px, py, pz),
        (qx, qy, qz, qw),
        rospy.Time.now(),
        "base",
        "odom"
    )

if __name__ == '__main__':
    rospy.init_node("odom2base")
    rospy.Subscriber("/mavros/odom_convert", Odometry, callback)
    rospy.spin()
