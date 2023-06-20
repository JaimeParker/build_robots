# Xavier NX 刷机

参考：

* https://zhuanlan.zhihu.com/p/635654880
* https://blog.csdn.net/m0_53717069/article/details/128536837
* https://developer.nvidia.com/embedded/learn/get-started-jetson-xavier-nx-devkit



适用于emmc系统，配置有ssd，并计划将ssd用于主系统。

注意，此操作需要一台主机（linux OS），一根usb-typec数据线（主机接到板子上），板子，以及查清楚板子如何进入Recovery模式。

首先下载[Nvidia SDK Manager](https://developer.nvidia.cn/sdk-manager)，注意，OS为18.04，安装的系统也只能是18.04，同理对于20系统。

其次按住Recovery按钮，给板子上电，上电后保持按住3~4秒后松手。

打开主机，输入`lsusb`命令查看连接的设备，应为：

```
Bus <bbb> Device <ddd>: ID 0955: <nnnn> Nvidia Corp.
```

其中，`<nnnn>`为7e19时，说明Xavier NX已进入Recovery模式并连接成功，否则排查错误。

此时可以进入SDK Manager：

**在第一个界面，STEP 01，有四个板块**，如果是20的主机，则选择JetPack5系列，18则选择4系列；注意Additional SDKS不要选择。

**第二个界面，STEP 02**，在Target Components里只选择Jetson Linux，其余的组件一概不要；之后开始下载；

**第三个界面，STEP 03**，弹出窗口*SDK Manaager is about to flash your Jetson Xavier NX moudle*，在第一项选择*Manual Setup*，确保第八项*Storage Device*为*EMMC/SD Card default*（对于国产）；之后flash；

**第四个界面，STEP 04，等**

至于设置用户名什么的，正常来就行。

之后会经过重启等操作，直到正式进入系统。接下来进行的操作是，将安装在emmc上的，通常只有8G或16G的系统，复制到ssd上去，并且完成ssd启动，也就是扩容。

* 如果ssd没有做分区，参考第二篇博客，https://blog.csdn.net/m0_53717069/article/details/128536837
* 如果ssd分区完成，继续

github上下载rootOnNVMe，拷贝到板子里；

```shell
sudo parted /dev/nvme0n1  # 进入parted
mklabel gpt  # 将ssd设置成gpt格式
mkpart logical 0 -1  # 将磁盘所有的容量设置为gpt格式
p  # 查看设置结果
```

上述参考合众恒跃说明书，NVIDIA官方授权店。

>我在进行这几步时出了些问题，主要是ssd上已有之前的18.04，最后提示我重启，并且重启后ssd直接掉了，在文件里找不到，不过没关系，继续这几步就可以。

输入命令格式化分区：

```shell
sudo mkfs.ext4 /dev/nvme0n1p1
```

然后可以进入拷贝操作，（如果之前这些不执行，且原来ssd上有18.04，这里就会提示说usr var等已经有了，无法copy）

```shell
cd rootOnNVMe
sudo ./copy-rootfs-ssd.sh
# 输入密码等操作
sudo ./ setup-service.sh
```

重启，测试，使用`df -h`命令，应该可以看到File System的`/dev/nvme0n1p1`被Mounted在了`~`下，且容量和ssd基本一致（系统本来也有7G左右）。

更多操作，[参考合众恒跃说明书](HZHY.pdf)，作者白新乐

