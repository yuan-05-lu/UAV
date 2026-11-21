#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from quadrotor_msgs.msg import PositionCommand
from mavros_msgs.msg import PositionTarget

class PosCmdBridge:
    def __init__(self):
        rospy.init_node("poscmd_bridge_to_px4")
        self.cmd_sub = rospy.Subscriber(
            "/planning/pos_cmd",
            PositionCommand,
            self.cmd_callback,
            queue_size=10,
        )
        self.setpoint_pub = rospy.Publisher(
            "/mavros/setpoint_raw/local",
            PositionTarget,
            queue_size=10,
        )
        self.last_cmd_time = 0.0
        self.got_cmd = False
        self.hold_x = 0.0
        self.hold_y = 0.0
        self.hold_z = 1.5
        self.hold_yaw = 0.0
        self.hold_vx = 0.0
        self.hold_vy = 0.0
        self.hold_vz = 0.0
        self.timeout = rospy.get_param("~timeout", 3.0)
        self.rate = rospy.Rate(100)

    def cmd_callback(self, msg):
        self.last_cmd_time = rospy.Time.now().to_sec()
        self.got_cmd = True
        self.hold_x = msg.position.x
        self.hold_y = msg.position.y
        self.hold_z = msg.position.z
        self.hold_yaw = msg.yaw
        self.hold_vx = msg.velocity.x
        self.hold_vy = msg.velocity.y
        self.hold_vz = msg.velocity.z

    def run(self):
        while not rospy.is_shutdown():
            if not self.got_cmd:
                self.rate.sleep()
                continue

            now = rospy.Time.now().to_sec()
            timed_out = (now - self.last_cmd_time) > self.timeout

            sp = PositionTarget()
            sp.header.stamp = rospy.Time.now()
            sp.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
            # 使用位置 + 速度 + yaw；不用加速度 / yaw_rate（MAVROS 按 ENU 转换）
            sp.type_mask = (
                PositionTarget.IGNORE_AFX
                | PositionTarget.IGNORE_AFY
                | PositionTarget.IGNORE_AFZ
                | PositionTarget.IGNORE_YAW_RATE
            )
            sp.position.x = self.hold_x
            sp.position.y = self.hold_y
            sp.position.z = self.hold_z
            sp.yaw = self.hold_yaw
            if timed_out:
                rospy.logwarn_throttle(
                    2.0, "pos cmd timeout, holding last position (vel=0)"
                )
                sp.velocity.x = 0.0
                sp.velocity.y = 0.0
                sp.velocity.z = 0.0
            else:
                sp.velocity.x = self.hold_vx
                sp.velocity.y = self.hold_vy
                sp.velocity.z = self.hold_vz

            self.setpoint_pub.publish(sp)
            self.rate.sleep()

if __name__ == "__main__":
    try:
        PosCmdBridge().run()
    except rospy.ROSInterruptException:
        pass