#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import tf
from nav_msgs.msg import Odometry
import tf.transformations as tft

br = tf.TransformBroadcaster()

def frd2enu_callback(msg):
    out = Odometry()
    out.header = msg.header
    out.header.frame_id = "odom"
    out.child_frame_id = "base_link"

    xf = msg.pose.pose.position.x
    yf = msg.pose.pose.position.y
    zf = msg.pose.pose.position.z

    out.pose.pose.position.x = xf
    out.pose.pose.position.y = -yf
    out.pose.pose.position.z = zf

    qx = msg.pose.pose.orientation.x
    qy = msg.pose.pose.orientation.y
    qz = msg.pose.pose.orientation.z
    qw = msg.pose.pose.orientation.w
    q_msg = [qx, qy, qz, qw]
    q_frd = q_msg
    q_rot_x = tft.quaternion_about_axis(3.14159265, (1, 0, 0))
    q_flu = tft.quaternion_multiply(q_frd, q_rot_x)
    #q_rot_z = tft.quaternion_about_axis(1.5707963, (0, 0, 1))
    #q_flu = tft.quaternion_multiply(tft.quaternion_multiply(q_rot_z, q_rot_x), q_frd)

    out.pose.pose.orientation.x = q_flu[0]
    out.pose.pose.orientation.y = q_flu[1]
    out.pose.pose.orientation.z = q_flu[2]
    out.pose.pose.orientation.w = q_flu[3]

    vxf = msg.twist.twist.linear.x
    vyf = msg.twist.twist.linear.y
    vzf = msg.twist.twist.linear.z
    out.twist.twist.linear.x = vxf
    out.twist.twist.linear.y = -vyf
    out.twist.twist.linear.z = vzf
    out.twist.twist.angular = msg.twist.twist.angular

    pub.publish(out)

    br.sendTransform(
        (out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z),
        (out.pose.pose.orientation.x, out.pose.pose.orientation.y,
         out.pose.pose.orientation.z, out.pose.pose.orientation.w),
        rospy.Time.now(),
        "odom",
        "map"
    )

if __name__ == '__main__':
    rospy.init_node("pose_to_odom")
    sub = rospy.Subscriber("/mavros/local_position/odom", Odometry, frd2enu_callback)
    pub = rospy.Publisher("/mavros/odom_convert", Odometry, queue_size=10)
    rospy.spin()
