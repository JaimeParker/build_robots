# Ros Bridge with Nokov

在Ubuntu 20.04 上通过ros获取刚体位姿，速度，与加速度信息。

## 1. Connection

首先**在ubuntu上ping通windows**。

* windows打开专用网络共享，可在专用网络上被发现
* 关闭防火墙（后来测试发现不用关，但是win之间ping可能需要关）
* 确认网络ip地址

我们的连接是动捕占据了主机的网线连接，即以太网，因此只能通过WIFI向外发送；

首先找到此WIFI下的主机IPV4 IP地址，现主机为`192.168.3.15`，需要在Nokov中设置发送IP也为此IP；

* 在ubuntu中 `ping 192.168.3.15`，通则成功

## 2. Ros Topic

确认网络连接无误后，在ubuntu上打开vrpn server，使用roslaunch方式（如果改变ip地址，修改launch的IP地址）；

将获得pose, twist, accel三个话题；

以pose为例：

* topic type: geometry_msgs/**PoseStamped**
* topic frequency: 60 hz
* frame_id: world
