基于 FAST-LIVO2 的激光雷达-惯性-视觉融合 SLAM 系统

## 快速开始

### 1. 安装 ROS2 Humble
```bash
sudo apt update
sudo apt install ros-humble-desktop python3-colcon-common-extensions
```

### 2. 编译工作空间
```bash
source /opt/ros/humble/setup.bash
./build_all_workspaces.sh
```

### 3. 运行系统
```bash
./start_all.sh      # 启动所有节点
./stop_all.sh       # 停止所有节点
```

## 项目结构

```
.
├── build_all_workspaces.sh   # 编译脚本
├── start_all.sh              # 启动脚本
├── stop_all.sh               # 停止脚本
├── livox_ws/                 # Livox 雷达驱动
├── fastlivo2_ws/             # FAST-LIVO2 算法
├── Livox-SDK2/               # Livox SDK
└── docs/                     # 文档
```

## 硬件要求

- 激光雷达: Livox Mid360
- 相机: RealSense D435i (可选)
- 系统: Ubuntu 22.04 + ROS2 Humble

## 详细文档

详见 [docs/](docs/) 目录。

## 许可证

GPLv2
