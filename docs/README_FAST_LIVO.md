# FAST-LIVO 快速启动指南

## 环境要求

- ROS 2 Humble
- Livox Mid-360 雷达
- RealSense D435i 相机

## 一键启动

### 启动所有节点

在 `/home/longkang/LV` 目录下运行：

```bash
./start_all.sh
```

此脚本会在**后台启动所有节点**：

1. **Livox Driver** - 启动 Livox Mid-360 驱动
2. **RealSense Camera** - 启动 RealSense 相机
3. **FAST-LIVO** - 启动 FAST-LIVO 算法

所有输出会重定向到日志文件，方便调试。

### 查看实时日志

启动后可以查看各节点的实时日志：

```bash
# 查看 Livox 驱动日志
tail -f logs/livox_driver.log

# 查看 RealSense 相机日志
tail -f logs/realsense.log

# 查看 FAST-LIVO 日志
tail -f logs/fastlivo.log
```

按 `Ctrl+C` 退出日志查看，不会影响节点运行。

### 停止所有节点

在 `/home/longkang/LV` 目录下运行：

```bash
./stop_all.sh
```

此脚本会通过 PID 文件精确停止所有后台节点。

## 手动启动（可选）

如果需要单独启动各个节点：

### 1. 启动 Livox 驱动

```bash
cd /home/longkang/LV/livox_ws
source install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

### 2. 启动 RealSense 相机

```bash
source /opt/ros/humble/setup.bash
ros2 launch realsense2_camera rs_launch.py rgb_camera.profile:=640x480x30 align_depth.enable:=false
```

### 3. 启动 FAST-LIVO

```bash
cd /home/longkang/LV
source /opt/ros/humble/setup.bash
source livox_ws/install/setup.bash
source fastlivo2_ws/install/setup.bash
ros2 launch fast_livo mapping_mid360.launch.py
```

## 配置文件

- Livox 配置: `livox_ws/src/livox_ros_driver2/config/MID360_config.json`
- FAST-LIVO 配置: `fastlivo2_ws/src/FAST-LIVO2/config/mid360.yaml`

## 启动 RViz 查看

在新的终端中：

```bash
source /opt/ros/humble/setup.bash
rviz2
```

然后加载 FAST-LIVO 的配置文件（如有）。

## 注意事项

1. 确保雷达和相机已正确连接
2. 启动顺序很重要：先启动驱动，再启动算法
3. 如果遇到问题，先运行 `stop_all.sh` 清理进程，然后重新启动
4. FAST-LIVO 的配置中 `lidar_type: 1`（使用 CustomMsg）已正确设置

## 故障排查

### 雷达无数据

检查 `/livox/lidar` 话题是否有数据：
```bash
ros2 topic hz /livox/lidar
```

### IMU 无数据

检查 `/livox/imu` 话题：
```bash
ros2 topic hz /livox/imu
```

### FAST-LIVO 无法初始化

1. 确认配置文件中 `lidar_type: 1`
2. 检查 IMU 频率（应为约 200Hz）
3. 确保点云数据不为空
4. 检查雷达前方 0.5 米内无遮挡（盲区设置）

## 修改说明

**重要修改**：
- 将 `mid360.yaml` 中的 `lidar_type` 从 `8` 改为 `1`
- 原因：FAST-LIVO 需要订阅 `livox_ros_driver2::msg::CustomMsg` 而非 `sensor_msgs::msg::PointCloud2`
- 这解决了算法无法初始化的问题
