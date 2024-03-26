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

## stdgpu

上述GUN问题是在编译stdgpu时出现的，nvblox下载的是1.13.0 release，本人使用相同版本，将GUN升级后使用同样的方式编译，我编译不过，报错：

<pre><font color="#4E9A06"><b>b121-legion@b121legion-LEGION-REN9000K-34IRZ</b></font>:<font color="#3465A4"><b>~/3rdParty/stdgpu-1.3.0</b></font>$ cmake --build build --config Release --parallel 10
[  3%] Built target gtest
[ 16%] Built target stdgpu
[ 19%] Built target gtest_main
[ 23%] Built target contract
[ 26%] Built target createAndDestroyDeviceArray
[ 30%] Built target createAndDestroyDeviceObject
[ 33%] Built target ranges
[ 37%] Built target atomic
[ 41%] Built target bitset
[ 44%] Built target mutex_array
[ 48%] Built target deque
[ 51%] Built target iterator
[ 55%] Built target unordered_map
[ 58%] Built target vector
[ 62%] Built target unordered_set
[ 64%] <font color="#4E9A06">Building CUDA object test/stdgpu/CMakeFiles/teststdgpu.dir/cuda/unordered_map.cu.o</font>
[ 66%] <font color="#4E9A06">Building CUDA object test/stdgpu/CMakeFiles/teststdgpu.dir/cuda/unordered_set.cu.o</font>
<b>/home/b121-legion/3rdParty/stdgpu-1.3.0/test/stdgpu/../stdgpu/unordered_datastructure.inc(2216)</b>: <font color="#CC0000"><b>error</b></font>: namespace <b>&quot;thrust&quot;</b> has no member &quot;<b>sort</b>&quot;

<b>/home/b121-legion/3rdParty/stdgpu-1.3.0/test/stdgpu/../stdgpu/unordered_datastructure.inc(2217)</b>: <font color="#CC0000"><b>error</b></font>: namespace <b>&quot;thrust&quot;</b> has no member &quot;<b>sort</b>&quot;

<b>/home/b121-legion/3rdParty/stdgpu-1.3.0/test/stdgpu/../stdgpu/unordered_datastructure.inc(2216)</b>: <font color="#CC0000"><b>error</b></font>: namespace <b>&quot;thrust&quot;</b> has no member &quot;<b>sort</b>&quot;

<b>/home/b121-legion/3rdParty/stdgpu-1.3.0/test/stdgpu/../stdgpu/unordered_datastructure.inc(2217)</b>: <font color="#CC0000"><b>error</b></font>: namespace <b>&quot;thrust&quot;</b> has no member &quot;<b>sort</b>&quot;

2 errors detected in the compilation of &quot;/home/b121-legion/3rdParty/stdgpu-1.3.0/test/stdgpu/cuda/unordered_set.cu&quot;.
2 errors detected in the compilation of &quot;/home/b121-legion/3rdParty/stdgpu-1.3.0/test/stdgpu/cuda/unordered_map.cu&quot;.
make[2]: *** [test/stdgpu/CMakeFiles/teststdgpu.dir/build.make:336: test/stdgpu/CMakeFiles/teststdgpu.dir/cuda/unordered_set.cu.o] Error 2
make[2]: *** Waiting for unfinished jobs....
make[2]: *** [test/stdgpu/CMakeFiles/teststdgpu.dir/build.make:321: test/stdgpu/CMakeFiles/teststdgpu.dir/cuda/unordered_map.cu.o] Error 2
make[1]: *** [CMakeFiles/Makefile2:705: test/stdgpu/CMakeFiles/teststdgpu.dir/all] Error 2
make: *** [Makefile:146: all] Error 2
</pre>

奇怪的是，通过nvblox的make命令，居然毫无问题地安装并运行了。

在[stdgpu官网build from source](https://stotko.github.io/stdgpu/getting_started/building_from_source.html)，可以看到其要求 CUDA11.0，而我用的是12.0，毕竟11.0实在是太老了。可能是CUDA问题导致找不到这个头文件了。

但是根据官网build from source，换成了 git clone版本，编译完美通过，真神奇。