# PX4-Autopilot make debug

* [main branch](https://github.com/PX4/PX4-Autopilot/tree/main)
* [v1.12.3](https://github.com/PX4/PX4-Autopilot/tree/v1.12.3)

## 0. Common Dependencies

* Eigen 3.8 or 3.9
* OpenCV 3 or 4

## 1. main or release(recommand)

推荐使用当前版本或stable及release版本；

### 1.0 Build and Config

要clone当前最新（debug）并且下载submodules：

```shell
git clone git@github.com:PX4/PX4-Autopilot.git --recursive
```

更推荐的做法是，使用release的分支：

```shell
git clone --branch release/1.13 git@github.com:PX4/PX4-Autopilot.git --recursive
```

branch后加的分支还可以是v1.13.3等。经测试，`release/1.13`可以正常运行。

之后运行配置脚本：

```shell
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
```

但是这个sh配置文件可能在别的版本中不在这个位置，需要查找。

之后**make**

```shell
make px4_sitl gazebo
```

成功则继续，失败则回去继续。

之后是向路径中添加

```sh
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
source ~/PX4-Autopilot/Tools/setup_gazebo.bash ~/PX4-Autopilot ~/PX4-Autopilot/build/px4_sitl_default
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:~/PX4-Autopilot
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:~/PX4-Autopilot/Tools/sitl_gazebo
```

根据实际情况配置；

之后再打开终端，会弹出下列消息：

```
GAZEBO_PLUGIN_PATH :/home/hazyparker/PX4-Autopilot/build/px4_sitl_default/build_gazebo
GAZEBO_MODEL_PATH :/home/hazyparker/PX4-Autopilot/Tools/sitl_gazebo/models
LD_LIBRARY_PATH /home/hazyparker/catkin_ws/devel/lib:/opt/ros/noetic/lib:/opt/ros/noetic/lib/x86_64-linux-gnu:/home/hazyparker/PX4-Autopilot/build/px4_sitl_default/build_gazebo
```

就基本成功，这些是**setup_gazebo.bash**中`echo`出来的，强迫症可以注释掉。

### 1.1 EV(External Vision)

在早期PX4版本中，使用`EKF2_AID_MASK`参数选择位姿估计模式，使用`EKF2_HDG_MODE`选择高度估计模式；但在新版本（不知道源自哪个版本开始）使用`EKF2_EV_CTRL`，来选择是否辅助定位，我暂时没找到新版本如何完全使用视觉定位。这在实际使用时并不影响，但在仿真时则难以判断视觉定位话题传递的成功与否以及效果好坏。

其中，`EKF2_AID_MASK=24`时为采用视觉定位和视觉偏航角。

## 2. v1.12.3

make for gazebo

```shell
git clone git@github.com:PX4/PX4-Autopilot.git
cd PX4-Autopilot
git checkout -b v1.12.3
git submodule update --init --recursive
sh Tools/setup/ubuntu.sh
make px4_sitl_default gazebo
```

make 之后，会有一些缺少的python3包，根据提示使用pip3安装即可；如果错误都是缺少包，依次安装即可。

### 2.1 Bug opencv2/aruco

* https://discuss.px4.io/t/make-px4-sitl-gazebo-fails-with-opencv-aruco-hpp/33007/3

表现为`gazebo_aruco_plugin.h`找不到opencv中的aruco这个包，这是opencv的额外包；

这个肯定是opencv的版本问题，但我试了4.4，4.3都不行；即使上述帖子说是4.3可以；

因此可以直接把相对应的cmakelist中aruco部分注释掉，make能过，但是可能会对gazebo的camera插件带来一些问题。

## 3. XTDrone

```shell
git clone https://github.com/PX4/PX4-Autopilot.git
mv PX4-Autopilot PX4_Firmware
cd PX4_Firmware
git checkout -b xtdrone/dev v1.11.0-beta1
git submodule update --init --recursive
make px4_sitl_default gazebo
```

or download from: https://www.yuque.com/xtdrone/manual_cn/basic_config

但是经测试也是一堆bug，暂时没解决。