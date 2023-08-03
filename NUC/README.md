# Intel NUC Config

没什么好说的，直接双系统就行。

**WIFI问题**

F2进BIOS

选择minimum install+不安装第三方驱动后，没有WIFI驱动，正在解决；

* [回答](https://askubuntu.com/questions/1402766/no-hope-for-ax211-wifi-working-on-ubuntu-20-04)，[下载文件](http://archive.ubuntu.com/ubuntu/pool/universe/b/backport-iwlwifi-dkms/)，**解决**

* 不知道有没有用https://launchpad.net/ubuntu/+source/backport-iwlwifi-dkms/8324-0ubuntu1

* 现在用的Intel NUC竞技场峡谷i7，https://item.jd.com/10077138237005.html#crumb-wrap

* 参考https://blog.csdn.net/qq_45488453/article/details/108453631

* 参考https://download.csdn.net/download/weixin_38724663/14048620?spm=1001.2101.3001.6661.1&utm_medium=distribute.pc_relevant_t0.none-task-download-2%7Edefault%7ECTRLIST%7EPaid-1-14048620-blog-108453631.235%5Ev38%5Epc_relevant_yljh&depth_1-utm_source=distribute.pc_relevant_t0.none-task-download-2%7Edefault%7ECTRLIST%7EPaid-1-14048620-blog-108453631.235%5Ev38%5Epc_relevant_yljh&utm_relevant_index=1

**显卡问题**

NUC用的是独显，Linux没找到对应的驱动，只有支持ubuntu22的，问题是代码和ros都是20的；所以rviz的fps只有1，几乎不能用；