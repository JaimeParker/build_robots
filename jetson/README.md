# jetson config

关于NVIDIA jetson，板载计算机，ARM64架构，的配置。

**为什么要用板载计算机？板载计算机和单片机的关系**

* 普通的MCU（如STM32）往往执行[下位机](https://baike.baidu.com/item/%E4%B8%8B%E4%BD%8D%E6%9C%BA/9624953)的任务；并不具有很强的算力，对于大型的建图导航可能不能够适用。
* 正常的遥控无人机，也只是飞控+下位机，没有如板载计算机这样的上位机，也就是很多智能的功能不能实现。
* NVIDIA的板载计算机也有很多个版本，根据算力不同（有适合跑深度学习的版本，也有简单SLAM的版本）。

* [NVIDIA Jetson维基百科](https://en.wikipedia.org/wiki/Nvidia_Jetson)，有几个详细的参数表。

## from 0 to 1 从到货到应用

jetson自带emmc系统（架构？），有的是插sd卡刷ubuntu系统，也有的似乎是emmc直接刷ubuntu系统（有一种安双系统时的与windows共存的感觉）；

对于Jetson nano而言，emmc的存储空间很小，往往装上ubuntu（18.04）之后就没有几个G的空间，因此需要外插sd卡，在sd卡上装新的ubuntu系统，跑ros等。

## system 系统

对于jetson nano，见到的是u盘作外接大规模存储系统（大于64G），这种情况下运行u盘一般会发烫，有一定的问题（不过都这样）。

## power 供电

对于`wheeltec`的无人车，Jetson nano在车辆运行时使用电池供电，但是该UGV要求给ros的供电电压不能小于10v，也有可能时nano对电压的要求就是不能小于10v，正常运行过程中在12v左右。

供电时可以接电源接口或直接usb供电，选择usb-micro usb的供电线。对接口的电压和供电线的最大电流都有要求，如果使用充电器的话需要仔细查看充电器能够实现的电压和电流。

## Remote Control 远程控制

当ubuntu安装完成后，实现对桌面的远程控制有很多种方法。

一种是不使用远程控制，一般在开发过程中，直接使用HDMI或DP外接显示器即可。

另一种则是使用远程控制，大致有几种方法。

### RC-nomachine

由于ARM64架构对于很多deb包不支持，因此很多远程控制（桌面控制）软件如向日葵等无法使用，但是基于局域网桌面控制的nomachine却支持ARM64架构。

[No Machine ARM下载地址](https://downloads.nomachine.com/linux/?id=30&distro=Arm)

之后进行正常配置，主控与板载计算机连接同一WIFI，输入板载ubuntu系统账户名和密码，调整视频参数即可。

### SSH 

- [ ] 补充SSH配置
