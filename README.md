# Visual-Exploration-Topic-2

面向弱纹理环境下的多传感器融合的鲁棒定位技术

## Vins-Fusion-Ros2

### 编译

使用 Vins-Fusion-Ros2版本执行，在整个编译过程中请先安装`thirdparty`中的第三方库
```shell
cd ..
cd thirdparty/ceres-solver/
mkdir -p build
cd build
cmake ..
make -j4
make install
```
然后对整个项目进行编译，执行 `colcon build`。
需要注意的是，可能受限于电脑性能等问题，无法一次性编译成功，甚至由于资源消耗过大导致退出和停止，这点是很正常的，尝试控制编译过程中并行构建任务的数量

```shell
colcon build --parallel-workers 2

```

当然，其中也可能由于编译进度不一致导致部分依赖未及时建立，以至于没有成功实现整体编译。建议多编译几次测试。

### 执行

对于构建后的包，会包含一个功能包`vins` 和一个可执行文件 `vins_node`。你或许会在使用

```shell

$ ros2 pkg executables vins
-> vins vins_node
```

发现还有一个额外的`vins` 文件可执行，但是其是无用的。
以选择`Vins Mono`配置为例，执行

```shell

ros2 run vins vins_node /workspace/src/VINS-Fusion-ROS2/config/euroc/euroc_mono_imu_config.yaml
```

其中会启动多个节点， 会包含多个话题 `/cam0/image_raw` 和 `/imu0` 分别是向其中输入图像和IMU数据的话题名称，可以直接将硬件端数据向话题中输入，会正常执行。
执行

```shell
ros2 launch vins vins_rviz.launch.xml
```

对于linux操作系统用户，一般需要额外处理桌面权限，方便直接展示Rviz2,在主机终端执行

```shell
xhost +local:docker
```

### 环境设置

使用Ubuntu 22.04，ros2-humble，仓库中给出了docker相关配置，可以使用`devcontainer`来实现在vscode上的整体docker环境开发。
此外，如遇无法拉取镜像的问题，可以咨询本人，通过获取压缩包来获取镜像。
