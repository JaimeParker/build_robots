# PX4 commands

## MC Rate control 

多旋翼角速度控制。

* Topic: `/mavros/setpoint_raw/attitude`
* message type: `mavros_msgs/PositionTarget`
* header: `mavros_msgs/AttitudeTarget.h`
* detail: [mavros_msgs/AttitudeTarget.msg](https://docs.ros.org/en/api/mavros_msgs/html/msg/AttitudeTarget.html)

注意，`setpoint_raw/target_attitude`是 PX4 loopback 回来的，是一个只能接收的话题。

还没来得及去看PX4源码，暂时记录一下现象：

* 设置body_rate+orientation，能保证姿态环跟的很好
* 设置body_rate，看控制器性能和body_rate_req的导数大小
* 只设置orientation，发现偏航角不转，且一段时间后系统失稳，拉不回来
* 设置orientation+body_rate.z，尽管实现了姿态+偏航角控制，但是姿态角的实际值和期望值差距很大，远没有body_rate+orientation时精确
