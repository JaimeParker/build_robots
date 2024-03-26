# nvblox

Signed Distance Functions (SDFs) on NVIDIA GPUs.

A GPU SDF library which offers

* GPU accelerated algorithms such as:
  * TSDF construction
  * Occupancy mapping
  * ESDF construction
  * Meshing
* ROS 2 interface (see [isaac_ros_nvblox](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox))
* Support for storage of various voxel types, and easily extended to custom voxel types.

Above we show reconstruction using data from the [3DMatch dataset](https://3dmatch.cs.princeton.edu/), specifically the [Sun3D](http://sun3d.cs.princeton.edu/) `mit_76_studyroom` scene. 

特别对NIVDIA的Jetson Xavier有支持，适合直接用在 NVIDIA 的板子上。

## Native Installation

Recommend install CUDA firstly, from [CUDA Toolkit 12.0](https://developer.nvidia.com/cuda-12-0-0-download-archive) or other suggested version.

- gtest
- glog
- gflags
- SQLite 3
- CUDA 11.0 - 11.8 (others might work but are untested)
- Eigen (no need to explicitly install, a recent version is built into the library)
- stdgpu (downloaded during compilation)
  Please run

```
sudo apt-get install -y libgoogle-glog-dev libgtest-dev libgflags-dev python3-dev libsqlite3-dev libbenchmark-dev
cd /usr/src/googletest && sudo cmake . && sudo cmake --build . --target install
```

I met GNU version error, which is too old, so I need to upgrade my GNU (gcc and g++), here is instruction: [How to Upgrade GCC on Ubuntu](https://webhostinggeeks.com/howto/how-to-upgrade-gcc-on-ubuntu/)

After that, cmake config and make successfully.

About installing Open3d, do not use pip, follow instruction [open3d website](https://www.open3d.org/) , and download the 0.16.0 version [here](https://github.com/isl-org/Open3D/releases/download/v0.16.0/open3d-app-0.16.0-Ubuntu.deb).

Then the example was run successfully.
