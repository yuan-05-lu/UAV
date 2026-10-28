#!/bin/bash
echo "==== 无人机开始起飞 ===="
rostopic pub /mavros/setpoint_position/local geometry_msgs/PoseStamped "{pose: {position: {x: 0, y: 0, z: 1.5}}}" --rate 20

