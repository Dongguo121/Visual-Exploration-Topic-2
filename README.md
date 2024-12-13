# Visual-Exploration-Topic-2

面向弱纹理环境下的多传感器融合的鲁棒定位技术

## Vins-Fusion-Ros2

### 环境

采用docker,直接使用vscode的devcontainer功能，本身在镜像中完成了整体的环境配置，无需担心，存在问题请和我联系

### 编译

进入`ros2_ws`，进行ros2编译

``` shell
cd ros2_ws
colcon build
```

在本次构建中应该不会出现问题
ros2_ws中只应该存放ros2包，其他第三方文件请存放在其外部。

### 改写

对于只使用vins_mono部分，所以在启动的时候直接使用

```shell
cd ros2_ws
source install/setup.bash
ros2 run vins vins_node path_to_`VINS-Fusion-ROS2/config/euroc/euroc_mono_imu_config.yaml`
```

对于其中可接受的话题为
`/cam0/image_raw`和`/imu0`可以参考这个发送内容，数据类型可以参考`sensor`中的内容。

额外的可以通过`rviz`来查看运动轨迹，存在问题请和我联系

