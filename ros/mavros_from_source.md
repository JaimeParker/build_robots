# Install Mavros and MAVLink

两种方式，源码和apt；没有开发mavros的需求则直接apt；

## Build Mavros from Source

**注意！！！：版本为noetic，ubuntu20.04**

ros安装完后，需要

```sh
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
sudo bash ./install_geographiclib_datasets.sh
```

如果报错

```sh
line 33 ... xxx-get-geoids: not found
line 37 ... xxx-datasets-download: not found
```

则运行

```sh
sudo apt-get install libgeographic-dev
sudo apt-get install geographiclib-tools
```

之后再运行sh文件即可。

参考[ROS with MAVROS Installation Guide](https://docs.px4.io/v1.12/en/ros/mavros_installation.html)，首先安装python-ros工具（ros noetic installation wiki不包含）：

```sh
sudo apt-get install python3-catkin-tools python3-rosinstall-generator -y
```

为mavros包单独创建一个工作空间，或者说是专门为wstool创建一个工作空间：

```sh
mkdir -p ~/mavros_ws/src
cd ~/mavros_ws
catkin init
wstool init src
```

```
wstool init ~/catkin_ws/src
```

安装mavlink：

```sh
rosinstall_generator --rosdistro noetic mavlink | tee /tmp/mavros.rosinstall
```

由于网络问题，这一步可能需要试很多次；

安装mavros：

```sh
rosinstall_generator --upstream mavros | tee -a /tmp/mavros.rosinstall
```

创建工作空间等：

```sh
wstool merge -t src /tmp/mavros.rosinstall
wstool update -t src -j4
rosdep install --from-paths src --ignore-src -y
```

Install [GeographicLib (opens new window)](https://geographiclib.sourceforge.io/)datasets:

```sh
./src/mavros/mavros/scripts/install_geographiclib_datasets.sh
```

Build:

```sh
catkin build
```

写入`~/.bashrc`

```sh
gedit ~/.bashrc
```

写入：

```
source ~/mavros_ws/devel/setup.bash
```

---

## Install Mavros via apt-get

标准流程：

```sh
sudo apt-get install ros-noetic-mavros ros-noetic-mavros-extras
```

据测试但不完全测试，上述源码安装方式不包含extras包，需要单独apt安装extras包，即单独运行：

```sh
sudo apt-get install ros-noetic-mavros-extras
```

