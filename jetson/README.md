# jetson config

## contents

NVIDIA的Jetson系列有多款产品，根据算力和尺寸等参数进行选择。

* [Jetson nano配置](Nano.md)
* [Jetson Xavier NX配置](XavierNX.md)

## WIFI config

一般是选择接免驱网卡的形式，即插即用。

连接WIFI需要按照以下步骤：

* 右上角的WIFI图标，右键，找到edit connections
* 进入后，可以看到已经建立过的connection，左下角找加号，新建一个连接
* 类型选择WIFI，选择client，SSID为WIFI名，而connection name是你对建立的这个连接的命名，与WIFI名无关，但是建议设置成有关联的名字
* 其他的一般不用管，可以在第一项选项中设置是否开机自动连这个网络，这样可以方便我们在户外使用热点控制
