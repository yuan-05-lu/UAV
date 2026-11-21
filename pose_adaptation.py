#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
import tf2_ros.static_transform_broadcaster

class OdomPassTestNode:
    def __init__(self):
        rospy.init_node("odom_pass_test_node")

        # 发布静态TF树 world->map, map->odom, base_link->link
        self.publish_static_tf_tree()

        # 订阅MAVROS原生odom
        self.odom_sub = rospy.Subscriber(
            "/mavros/local_position/odom",
            Odometry,
            self.odom_callback,
            queue_size=1
        )
        # 转发给Ego，话题保持 /convert
        self.convert_pub = rospy.Publisher("/mavros/odom_convert", Odometry, queue_size=10)
        self.tf_br = tf2_ros.TransformBroadcaster()
        rospy.loginfo("===== 验证脚本启动：纯透传无轴变换 + 内置静态TF =====")

    def publish_static_tf_tree(self):
        """world->map、map->odom、base_link->link，仿真全0单位旋转"""
        static_br = tf2_ros.StaticTransformBroadcaster()
        now = rospy.Time.now()
        specs = [
            ("world", "map", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
            ("map", "odom", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
            # 如果雷达SDF里pose是 0.3 0 0.1，就改成 (0.3, 0.0, 0.1)
            ("base_link", "link", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
        ]
        msgs = []
        for parent, child, trans, rot in specs:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = parent
            t.child_frame_id = child
            t.transform.translation.x = trans[0]
            t.transform.translation.y = trans[1]
            t.transform.translation.z = trans[2]
            t.transform.rotation.x = rot[0]
            t.transform.rotation.y = rot[1]
            t.transform.rotation.z = rot[2]
            t.transform.rotation.w = rot[3]
            msgs.append(t)
        static_br.sendTransform(msgs)
        rospy.loginfo("Static TF published: world->map->odom, base_link->link")

    def odom_callback(self, msg):
        # 原样转发到 /convert
        self.convert_pub.publish(msg)

        # 广播动态TF odom -> base_link，直接取自odom消息，不做变换
        tf_msg = TransformStamped()
        tf_msg.header.stamp = msg.header.stamp
        tf_msg.header.frame_id = "odom"
        tf_msg.child_frame_id = "base_link"

        tf_msg.transform.translation.x = msg.pose.pose.position.x
        tf_msg.transform.translation.y = msg.pose.pose.position.y
        tf_msg.transform.translation.z = msg.pose.pose.position.z
        tf_msg.transform.rotation = msg.pose.pose.orientation

        self.tf_br.sendTransform(tf_msg)

if __name__ == "__main__":
    try:
        node = OdomPassTestNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
