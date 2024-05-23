# PX4 commands

## MC Rate control 

多旋翼角速度控制。

* Topic: `/mavros/setpoint_raw/attitude`
* message type: `mavros_msgs/PositionTarget`
* header: `mavros_msgs/AttitudeTarget.h`
* detail: [mavros_msgs/AttitudeTarget.msg](https://docs.ros.org/en/api/mavros_msgs/html/msg/AttitudeTarget.html)

注意，`setpoint_raw/target_attitude`是 PX4 loopback 回来的，是一个只能接收的话题。

