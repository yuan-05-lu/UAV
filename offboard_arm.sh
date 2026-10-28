#!/bin/bash
echo "==== 尝试切换到 OFFBOARD 板外模式 ===="
rosservice call /mavros/set_mode "custom_mode: 'OFFBOARD'"
sleep 0.8
echo "==== 解锁电机 ARM ===="
rosservice call /mavros/cmd/arming "value: true"
echo "==== 执行完成，请查看QGC或者/mavros/state确认状态 ===="

