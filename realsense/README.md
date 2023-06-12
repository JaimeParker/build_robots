# Intel RealSense Config

## 1. get started

[get started website](https://www.intelrealsense.com/get-started)

get started界面，选择Depth camera（D400系列）或Tracking camera（T265）；

## 2. librealsense

[librealsense release](https://github.com/IntelRealSense/librealsense/releases)

在这里选择librealsense的版本进行安装，安装时要注意看相机是否被这个版本支持（T265的支持截至2.50.0）。

下面提供了两种安装方式：

* windows平台直接下载exe运行即可，或根据是否支持相机选择apt安装
* ubuntu下使用源码编译安装

### 2.1 exe or apt install

windows直接下载exe，点击运行；

linux下，如果最新版本涵盖了要用的相机，则可以使用官方文档的方式安装，即[Linux Distribution](https://github.com/IntelRealSense/librealsense/blob/development/doc/distribution_linux.md)：

- Register the server's public key:

```
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null
```

- Make sure apt HTTPS support is installed: `sudo apt-get install apt-transport-https`
- Add the server to the list of repositories:

```
echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
sudo tee /etc/apt/sources.list.d/librealsense.list
sudo apt-get update
```

- Install the libraries (see section below if upgrading packages):
  `sudo apt-get install librealsense2-dkms`
  `sudo apt-get install librealsense2-utils`
  The above two lines will deploy librealsense2 udev rules, build and activate kernel modules, runtime library and executable demos and tools.
- Optionally install the developer and debug packages:
  `sudo apt-get install librealsense2-dev`
  `sudo apt-get install librealsense2-dbg`
  With `dev` package installed, you can compile an application with **librealsense** using `g++ -std=c++11 filename.cpp -lrealsense2` or an IDE of your choice.

Reconnect the Intel RealSense depth camera and run: `realsense-viewer` to verify the installation.

Verify that the kernel is updated :
`modinfo uvcvideo | grep "version:"` should include `realsense` string.

这样的方式会安装最新版sdk并且每次update和upgrade时都会更新。只要确保相机被支持，就可以使用这种方式。

### 2.2 build from source

[Linux Ubuntu Installation](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md#building-librealsense2-sdk)

#### 2.2.1 Prerequisites

**Important:** Running RealSense Depth Cameras on Linux requires patching and inserting modified kernel drivers. Some OEM/Vendors choose to lock the kernel for modifications. Unlocking this capability may require modification of BIOS settings

**Make Ubuntu Up-to-date:**

- Update Ubuntu distribution, including getting the latest stable kernel:
  - `sudo apt-get update && sudo apt-get upgrade && sudo apt-get dist-upgrade`

**Note:** On stock Ubuntu 14 LTS systems and kernels prior to 4.4.0-04 the standard `apt-get upgrade` command is not sufficient to bring the distribution to the latest recommended baseline.
It is recommended to upgrade the distribution with:

- `sudo apt-get install --install-recommends linux-generic-lts-xenial xserver-xorg-core-lts-xenial xserver-xorg-lts-xenial xserver-xorg-video-all-lts-xenial xserver-xorg-input-all-lts-xenial libwayland-egl1-mesa-lts-xenial `
- Update OS Boot and reboot to enforce the correct kernel selection with
  `sudo update-grub && sudo reboot`
- Interrupt the boot process at Grub2 Boot Menu -> "Advanced Options for Ubuntu" and select the kernel version installed in the previous step. Press and hold SHIFT if the Boot menu is not presented.
- Complete the boot, login and verify that a supported kernel version (4.**[4,8,10,13,15,16]**]) is in place with `uname -r`

**Download/Clone librealsense github repository:**

- Get *librealsense* sources in one of the following ways:
  - Install *git* and download the complete source tree.
    `sudo apt-get install git`
    `git clone https://github.com/IntelRealSense/librealsense.git`
  - Download and unzip the latest stable version from `master` branch: https://github.com/IntelRealSense/librealsense/archive/master.zip

**Prepare Linux Backend and the Dev. Environment:**

1. Navigate to *librealsense* root directory to run the following scripts.
   Unplug any connected Intel RealSense camera.（确保相机未通过USB链接）

2. Install the core packages required to build *librealsense* binaries and the affected kernel modules:

   `sudo apt-get install libssl-dev libusb-1.0-0-dev libudev-dev pkg-config libgtk-3-dev cmake`

   这里注意cmake，（个人建议）不推荐最新版本源码安装或apt安装，而是使用3.24版本使用`./bootstrap`命令安装cmake；apt安装的版本低的cmake可能会在某些库出现问题，并且apt后再源码编译安装的cmake，可能对之前安装的某些库无法找到，比较麻烦。（这也是一个值得研究和解决的点）因此为了方便，不建议这里安装cmake。

   Distribution-specific packages:

   - Ubuntu 18/20/22:
     `sudo apt-get install libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev at`

> **Cmake Note**: certain librealsense CMAKE flags (e.g. CUDA) require version 3.8+ which is currently not made available via apt manager for Ubuntu LTS.
> Go to the [official CMake site](https://cmake.org/download/) to download and install the application

**Note**:

- on graphic sub-system utilization:
  *glfw3*, *mesa* and *gtk* packages are required if you plan to build the SDK's OpenGL-enabled examples. The *librealsense* core library and a range of demos/tools are designed for headless environment deployment.
- `libudev-dev` installation is optional but recommended, when the `libudev-dev` is installed the SDK will use an event-driven approach for triggering USB detection and enumeration, if not the SDK will use a timer polling approach which is less sensitive for device detection.

1. Run Intel Realsense permissions script from librealsense root directory:
   `./scripts/setup_udev_rules.sh`

   *Notice: One can always remove permissions by running:* *`./scripts/setup_udev_rules.sh --uninstall`*

2. Build and apply patched kernel modules for:

- **Ubuntu 20/22 (focal/jammy) with LTS kernel 5.13, 5.15 **
  `./scripts/patch-realsense-ubuntu-lts-hwe.sh`
- 一般来说，ubuntu的版本都是18或20（截止2023年），但是在支持T265的2.50.0版本中，并没有上述sh文件
- **Ubuntu 14/16/18/20 with LTS kernel (< 5.13) **
  `./scripts/patch-realsense-ubuntu-lts.sh`
- **Arch-based distributions** 这是Arch架构的，对于板子需要
  - Install the [base-devel](https://www.archlinux.org/groups/x86_64/base-devel/) package group.
  - Install the matching linux-headers as well (i.e.: linux-lts-headers for the linux-lts kernel).
  - Navigate to the scripts folder
    `cd ./scripts/`
  - Then run the following script to patch the uvc module:
    `./patch-arch.sh`

#### 2.2.2 Build and Install

标准的cmake库安装流程；在这里写一遍：

**首先进入源代码目录，创建build文件夹**

```shell
cd librealsense-2.50.0
mkdir build
```

**进行cmake configuration**

```shell
cmake ..
```

报warning可以，一般不影响编译；

报错则根据错误类型排除错误即可，一般是缺少依赖，安装即可。

**make**

```shell
make -j16
```

j后面的数字指的是使用多少个线程编译，越多越快，视CPU能力而定。

**install**

```shell
sudo make install
```

**run**

```shell
realsense-viewer
```

使用此命令来打开Viewer。
