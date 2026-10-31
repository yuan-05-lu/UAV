#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from quadrotor_msgs.msg import PositionCommand
from mavros_msgs.msg import PositionTarget

class PosCmdBridge:
    def __init__(self):
        rospy.init_node("poscmd_bridge_to_px4")
        self.cmd_sub = rospy.Subscriber("/planning/pos_cmd", PositionCommand,
                                        self.cmd_callback, queue_size=1)
        self.setpoint_pub = rospy.Publisher("/mavros/setpoint_raw/local",
                                            PositionTarget, queue_size=10)
        self.last_cmd_time = 0.0
        self.hold_x = 0.0
        self.hold_y = 0.0
        self.hold_z = 1.5
        self.hold_yaw = 0.0
        self.timeout = 1.0
        self.rate = rospy.Rate(100)   # 与 traj_server 同频

    def cmd_callback(self, msg):
        self.last_cmd_time = rospy.Time.now().to_sec()
        # pos_cmd 是 ENU；FRAME_LOCAL_NED标记非机体系，mavros插件自动ENU→NED转换
        self.hold_x = msg.position.x
        self.hold_y = msg.position.y
        self.hold_z = msg.position.z
        self.hold_yaw = msg.yaw

    def run(self):
        while not rospy.is_shutdown():
            now = rospy.Time.now().to_sec()
            sp = PositionTarget()
            sp.header.stamp = rospy.Time.now()
            # PositionTarget无FRAME_LOCAL_ENU常量；FRAME_LOCAL_NED标记非机体系，MAVROS自动将输入ENU转为NED下发PX4
            sp.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
            sp.type_mask = (PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY |
                            PositionTarget.IGNORE_VZ | PositionTarget.IGNORE_AFX |
                            PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ |
                            PositionTarget.IGNORE_YAW_RATE)
            sp.position.x = self.hold_x
            sp.position.y = self.hold_y
            sp.position.z = self.hold_z
            sp.yaw = self.hold_yaw

            if now - self.last_cmd_time > self.timeout:
                rospy.logwarn_throttle(2.0, "pos cmd timeout, holding last position")

            self.setpoint_pub.publish(sp)
            self.rate.sleep()

if __name__ == "__main__":
    try:
        PosCmdBridge().run()
    except rospy.ROSInterruptException:
        pass
