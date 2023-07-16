# Planner

主要使用的是FAST LAB的ego planner和HKUST Aerial Group的fast planner。

观察一下其主要的启动launch文件，大同小异；

```xml
<launch>
  <!-- size of map, change the size inflate x, y, z according to your application -->
  <arg name="map_size_x" value="40.0"/>
  <arg name="map_size_y" value="40.0"/>
  <arg name="map_size_z" value=" 3.0"/>

  <!-- topic of your odometry such as VIO or LIO -->
  <arg name="odom_topic" value="/visual_slam/odom" />

  <!-- main algorithm params -->
  <include file="$(find ego_planner)/launch/advanced_param.xml">

    <arg name="map_size_x_" value="$(arg map_size_x)"/>
    <arg name="map_size_y_" value="$(arg map_size_y)"/>
    <arg name="map_size_z_" value="$(arg map_size_z)"/>
    <arg name="odometry_topic" value="$(arg odom_topic)"/>

    <!-- camera pose: transform of camera frame in the world frame -->
    <!-- depth topic: depth image, 640x480 by default -->
    <!-- don't set cloud_topic if you already set these ones! -->
    <arg name="camera_pose_topic" value="/pcl_render_node/camera_pose"/>
    <arg name="depth_topic" value="/pcl_render_node/depth"/>

    <!-- topic of point cloud measurement, such as from LIDAR  -->
    <!-- don't set camera pose and depth, if you already set this one! -->
    <arg name="cloud_topic" value="/pcl_render_node/cloud"/>

    <!-- intrinsic params of the depth camera -->
    <arg name="cx" value="321.04638671875"/>
    <arg name="cy" value="243.44969177246094"/>
    <arg name="fx" value="387.229248046875"/>
    <arg name="fy" value="387.229248046875"/>

    <!-- maximum velocity and acceleration the drone will reach -->
    <arg name="max_vel" value="2.0" />
    <arg name="max_acc" value="3.0" />

    <!--always set to 1.5 times grater than sensing horizen-->
    <arg name="planning_horizon" value="7.5" /> 

    <!-- 1: use 2D Nav Goal to select goal  -->
    <!-- 2: use global waypoints below  -->
    <arg name="flight_type" value="2" />
    
    <!-- global waypoints -->
    <!-- It generates a piecewise min-snap traj passing all waypoints -->
    <arg name="point_num" value="5" />

    <arg name="point0_x" value="-15.0" />
    <arg name="point0_y" value="0.0" />
    <arg name="point0_z" value="1.0" />

    <arg name="point1_x" value="0.0" />
    <arg name="point1_y" value="15.0" />
    <arg name="point1_z" value="1.0" />

    <arg name="point2_x" value="15.0" />
    <arg name="point2_y" value="0.0" />
    <arg name="point2_z" value="1.0" />

    <arg name="point3_x" value="0.0" />
    <arg name="point3_y" value="-15.0" />
    <arg name="point3_z" value="1.0" />

    <arg name="point4_x" value="-15.0" />
    <arg name="point4_y" value="0.0" />
    <arg name="point4_z" value="1.0" />
    
  </include>

  <!-- trajectory server -->
  <node pkg="ego_planner" name="traj_server" type="traj_server" output="screen">
    <remap from="/position_cmd" to="planning/pos_cmd"/>

    <remap from="/odom_world" to="$(arg odom_topic)"/>
    <param name="traj_server/time_forward" value="1.0" type="double"/>
  </node>

  <node pkg="waypoint_generator" name="waypoint_generator" type="waypoint_generator" output="screen">
    <remap from="~odom" to="$(arg odom_topic)"/>        
    <remap from="~goal" to="/move_base_simple/goal"/>
    <remap from="~traj_start_trigger" to="/traj_start_trigger" />
    <param name="waypoint_type" value="manual-lonely-waypoint"/>    
  </node>

  <!-- use simulator -->
  <include file="$(find ego_planner)/launch/simulator.xml">
    <arg name="map_size_x_" value="$(arg map_size_x)"/>
    <arg name="map_size_y_" value="$(arg map_size_y)"/>
    <arg name="map_size_z_" value="$(arg map_size_z)"/>
    <arg name="c_num" value="200"/>
    <arg name="p_num" value="200"/>
    <arg name="min_dist" value="1.2"/>

    <arg name="odometry_topic" value="$(arg odom_topic)" />
  </include>

  <include file="$(find ego_planner)/launch/rviz.launch"/>

</launch>
```

需要关注的为以下几个topic：

```xml
  <!-- topic of your odometry such as VIO or LIO -->
  <arg name="odom_topic" value="/visual_slam/odom" />
    <!-- camera pose: transform of camera frame in the world frame -->
    <!-- depth topic: depth image, 640x480 by default -->
    <!-- don't set cloud_topic if you already set these ones! -->
    <arg name="camera_pose_topic" value="/pcl_render_node/camera_pose"/>
    <arg name="depth_topic" value="/pcl_render_node/depth"/>

    <!-- topic of point cloud measurement, such as from LIDAR  -->
    <!-- don't set camera pose and depth, if you already set this one! -->
    <arg name="cloud_topic" value="/pcl_render_node/cloud"/>
```

其中，1是必须的，23和4则为任选一种；这几个话题的消息类型分别为：

* `odom_topic`: nav_msg::Odometry
* `camera_pose_topic` : geometry_msg::posestamp
* `depth_topic`: sensor_msg::Image
* `cloud_topic`: PointCloud2

所以在移植时需要注意话题消息类型对应，其中，这些话题消息的header.frame_id均为world；这一点比较麻烦，因为realsense的ros节点，生成的各种话题消息有其定义好的坐标系及tf tree转换关系；

转换起来有点麻烦，暂时我是直接新建立一个话题，通过一个subscribe和publisher完成接收消息，更改header.frame_id然后再发布出去，给planner的话题接收；

在移植时，为了方便我会将这一段直接注释掉：

```xml
  <!-- use simulator -->
  <include file="$(find ego_planner)/launch/simulator.xml">
    <arg name="map_size_x_" value="$(arg map_size_x)"/>
    <arg name="map_size_y_" value="$(arg map_size_y)"/>
    <arg name="map_size_z_" value="$(arg map_size_z)"/>
    <arg name="c_num" value="200"/>
    <arg name="p_num" value="200"/>
    <arg name="min_dist" value="1.2"/>

    <arg name="odometry_topic" value="$(arg odom_topic)" />
  </include>
```

也就是不用simulator生成的地图，直接用gazebo中的或者realsense获得的。

一些bugs：

* 在fast planner中， 运行rviz后real map默认使用的是激光雷达的话题，如果不对我们使用的点云做坐标系转换，则会导致由于坐标系不同的报错；但是即使是坐标系转换到world下，输入深度信息，其inflate map仍然不是我们想要的occupancy grid map格式，而是点云格式；
* 而在ego中，只要注释掉simulator和完成world的坐标转换，可以得到正确的inflate map和real map

FIXME：

* 我用的方法太愚蠢了，有空了标准化一下，使用标准的tf2完成这些转换。
* 一个可能出现的bug，rate的定义和queue size的选择，我记得是不匹配的话会有冲突
