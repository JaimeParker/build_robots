#### v 1.0

截至2023.8.11，进行完整ego planner实验的流程如下：

飞机初始位置确定好后，**上电**；NUC需要按电源开关，Xavier NX在上电之前最好拔掉飞控与NX之间的串口连接线，等Nomachine连接上后再连接；

打开QGC，连接数传，确定**通信情况**；

**赋予串口权限**；NUC是usb连接，NX是串口连接，用的口不一样：

```shell
# for nuc
sudo chmod 666 /dev/ttyACM0
# for NX
sudo chmod 666 /dev/ttyTHS0
```

*如何赋予串口永久权限是个遗留问题，尝试了很多方法没有解决*

启动**mavros和Pixhawk**之间的通信：

```shell
roslaunch mavros px4.launch
```

之后启动**相机和坐标系转换**部分：

```shell
# (default) for realsense d425i and t265 at the same time
roslaunch realsense2_camera rs_d400_and_t265.launch
# t265 tf to mavros
roslaunch vision_to_mavros t265_tf_to_mavros.launch
```

要特别注意相机节点会不会挂掉（红色ERROR），挂掉的话重开一次；

至此，视觉定位节点已经完全启动，无人机已经能满足切定点和视觉定位任务，有三种方式可以查看是否满足：

* 执行命令`rostopic echo /mavros/local_position/pose`，观察是否有输出
* 遥控器是否能够顺利切定点模式
* QGC的MAVLink Inspector中，`LOCAL_POSITION_NED`的`x,y,z`是否有输出，且输出是否稳定；（建议将时间区间调整至60秒，纵轴的量度可以自动，也可以根据观察精度调节）

下一步是将**位姿和地图话题转换**到ego planner要求的world坐标系下：

```shell
roslaunch px4_mavros_controller mavros_ego_planner.launch
```

**执行ego planner**：

```shell
roslaunch ego_planner test.launch
```

观察建图和坐标是否正常；

注意，由于我们设置的飞行高度不超过1.3m，因此会有一层粉丝色的平面盖在膨胀地图(inflate map)上，此时可以在Rviz左侧的参数栏取消inflate map的勾选，只留下普通的栅格地图（）

最后启动**板外模式（offboard）和起飞待命**：

```shell
rosrun ego_planner cmd_to_controller
```

此后飞机会先判断飞控连接状态，然后切offboard模式后自动起飞，等待Rviz中的终点坐标；

**提供终点坐标**有两种方式：

* 在Rviz界面，点击2D Nav Goal（粉紫色），在地图上单击一点按住并拖拽方向，提供一个带有偏航角的终点
* 在Rviz界面，按键盘G键，后在地图上点击一点，即为目标点，但不包含偏航角信息

最后，每次重新规划之前，这些节点最好都重新启动一遍；



**汇总**的命令如下：

```shell
sudo chmod 666 /dev/ttyxxx
roslaunch mavros px4.launch
roslaunch realsense2_camera rs_d400_and_t265.launch
roslaunch vision_to_mavros t265_tf_to_mavros.launch
roslaunch px4_mavros_controller mavros_ego_planner.launch
roslaunch ego_planner test.launch
```

最后一个命令，单独开一个终端启动，方便同时监督planner和最后一个命令终端的输出信息：

```shell
rosrun ego_planner cmd_to_controller
```

