# ROS
* [mavros px4 gazebo](mavros_px4_gazebo.md)

**ROS noetic installation**, *from* [wiki](http://wiki.ros.org/noetic/Installation)

Setup your computer to accept software from packages.ros.org.

```shell
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

Setup keys

```shell
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```

Install

```shell
sudo apt update
```

```shell
sudo apt install ros-noetic-desktop-full
```

Environment

```shell
source /opt/ros/noetic/setup.bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Dependencies

```shell
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
```

```shell
sudo apt install python3-rosdep
```

```shell
sudo rosdep init
rosdep update
```

