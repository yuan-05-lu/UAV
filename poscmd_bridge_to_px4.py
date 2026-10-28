#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from quadrotor_msgs.msg import PositionCommand
from mavros_msgs.msg import PositionTarget
from tf.transformations import quaternion_from_euler

class PosCmdBridge:
    def __init__(self):
        rospy.init_node("poscmd_bridge_to_px4")
        # 修改订阅类型为 PositionCommand
        self.cmd_sub = rospy.Subscriber("/planning/pos_cmd", PositionCommand, self.cmd_callback, queue_size=10)
        self.setpoint_pub = rospy.Publisher("/mavros/setpoint_raw/local", PositionTarget, queue_size=10)

        self.last_cmd_time = 0.0
        self.hold_x = 0.0
        self.hold_y = 0.0
        self.hold_z = 1.5
        self.hold_yaw = 0.0
        self.timeout = 0.3

        self.rate = rospy.Rate(20)

    def cmd_callback(self, msg):
        self.last_cmd_time = rospy.Time.now().to_sec()
        # PositionCommand 的位置字段
        self.hold_x = msg.position.y
        self.hold_y = msg.position.x
        self.hold_z = -msg.position.z
        self.hold_yaw = 0.0

    def run(self):
        while not rospy.is_shutdown():
            now = rospy.Time.now().to_sec()
            sp = PositionTarget()
            sp.header.stamp = rospy.Time.now()
            sp.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
            # type_mask：只使用位置控制，速度/加速度忽略
            sp.type_mask = PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ \
                           | PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ \
                           | PositionTarget.IGNORE_YAW_RATE

            # ENU -> NED转换
            sp.position.x = self.hold_x
            sp.position.y = self.hold_y
            sp.position.z = self.hold_z
            sp.yaw = self.hold_yaw

            self.setpoint_pub.publish(sp)
            self.rate.sleep()
    
    def cmd_callback(self, msg):
        rospy.loginfo("get pos cmd! x={}, y={}, z={}".format(msg.position.x, msg.position.y, msg.position.z))
        self.last_cmd_time = rospy.Time.now().to_sec()
        self.hold_x = msg.position.x
        self.hold_y = msg.position.y
        self.hold_z = msg.position.z
        self.hold_yaw = 0.0

if __name__ == "__main__":
    try:
        node = PosCmdBridge()
        node.run()
    except rospy.ROSInterruptException:
        pass
