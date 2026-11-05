#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
MAVROS odom -> /mavros/odom_convert + TF tree for Gazebo / RViz / EGO (ENU/FLU).

  world --static--> map --static--> odom --dynamic--> base_link --static--> link

Default: pass-through pose/twist from /mavros/local_position/odom (MAVROS already ENU).
Set ~convert_ned_to_enu:=true only if raw odom is still NED after you verified in flight.

Do not run odom2base.py; disable MAVROS publish_tf if it duplicates odom->base_link.
"""
import rospy
import tf
import tf2_ros
import tf.transformations as tft
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

br = tf.TransformBroadcaster()
pub = None


def ned_pos_to_enu(x, y, z):
    return y, x, -z


def ned_quat_to_enu(qx, qy, qz, qw):
    q_enu_ned = tft.quaternion_from_euler(1.57079632679, 0.0, 1.57079632679)
    q_out = tft.quaternion_multiply(q_enu_ned, [qx, qy, qz, qw])
    return q_out[0], q_out[1], q_out[2], q_out[3]


def publish_static_tf_tree():
    if not rospy.get_param("~publish_static_tf", True):
        return

    static_br = tf2_ros.StaticTransformBroadcaster()
    now = rospy.Time.now()

    map_tx = rospy.get_param("~map_to_odom/x", 0.0)
    map_ty = rospy.get_param("~map_to_odom/y", 0.0)
    map_tz = rospy.get_param("~map_to_odom/z", 0.0)
    map_yaw = rospy.get_param("~map_to_odom/yaw", 0.0)
    q_map_odom = tft.quaternion_from_euler(0.0, 0.0, map_yaw)

    link_tx = rospy.get_param("~base_link_to_link/x", 0.0)
    link_ty = rospy.get_param("~base_link_to_link/y", 0.0)
    link_tz = rospy.get_param("~base_link_to_link/z", 0.0)
    link_yaw = rospy.get_param("~base_link_to_link/yaw", 0.0)
    q_bl_link = tft.quaternion_from_euler(0.0, 0.0, link_yaw)

    specs = [
        ("world", "map", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
        ("map", "odom", (map_tx, map_ty, map_tz), q_map_odom),
    ]
    if rospy.get_param("~publish_base_link_to_link", True):
        specs.append(
            ("base_link", "link", (link_tx, link_ty, link_tz), q_bl_link)
        )

    msgs = []
    for parent, child, trans, quat in specs:
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = parent
        t.child_frame_id = child
        t.transform.translation.x = trans[0]
        t.transform.translation.y = trans[1]
        t.transform.translation.z = trans[2]
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        msgs.append(t)

    static_br.sendTransform(msgs)
    rospy.loginfo("pose_to_odom: static TF world->map->odom (optional base_link->link)")


def fill_odometry(msg, convert_ned):
    out = Odometry()
    out.header = msg.header
    out.header.frame_id = "odom"
    out.child_frame_id = "base_link"

    if not convert_ned:
        out.pose = msg.pose
        out.twist = msg.twist
        return out

    p = msg.pose.pose.position
    x, y, z = ned_pos_to_enu(p.x, p.y, p.z)
    out.pose.pose.position.x = x
    out.pose.pose.position.y = y
    out.pose.pose.position.z = z

    o = msg.pose.pose.orientation
    qx, qy, qz, qw = ned_quat_to_enu(o.x, o.y, o.z, o.w)
    out.pose.pose.orientation.x = qx
    out.pose.pose.orientation.y = qy
    out.pose.pose.orientation.z = qz
    out.pose.pose.orientation.w = qw

    v = msg.twist.twist.linear
    vx, vy, vz = ned_pos_to_enu(v.x, v.y, v.z)
    out.twist.twist.linear.x = vx
    out.twist.twist.linear.y = vy
    out.twist.twist.linear.z = vz
    out.twist.twist.angular = msg.twist.twist.angular
    return out


def odom_callback(msg):
    convert_ned = rospy.get_param("~convert_ned_to_enu", False)
    out = fill_odometry(msg, convert_ned)
    pub.publish(out)

    p = out.pose.pose.position
    o = out.pose.pose.orientation
    stamp = msg.header.stamp if msg.header.stamp.to_sec() > 0.0 else rospy.Time.now()

    br.sendTransform(
        (p.x, p.y, p.z),
        (o.x, o.y, o.z, o.w),
        stamp,
        "base_link",
        "odom",
    )


if __name__ == "__main__":
    rospy.init_node("pose_to_odom")
    publish_static_tf_tree()
    pub = rospy.Publisher("/mavros/odom_convert", Odometry, queue_size=10)
    rospy.Subscriber(
        "/mavros/local_position/odom",
        Odometry,
        odom_callback,
        queue_size=10,
    )
    rospy.spin()
