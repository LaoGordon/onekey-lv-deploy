# LV项目

基于FAST-LIVO2的激光雷达-惯性-视觉融合SLAM系统

## 项目简介

本项目是一个完整的LiDAR-Inertial-Visual融合SLAM系统，基于FAST-LIVO2开发，支持实时三维重建和机器人定位。

## 项目结构

```
LV/
├── build_all_workspaces.sh          # 自动编译脚本
├── docs/                            # 项目文档目录
│   ├── README.md                    # 文档目录说明
│   ├── BUILD_WORKSPACES.md         # 工作空间编译指南
│   ├── README_FAST_LIVO.md         # FAST-LIVO2详细说明
│   ├── DAILY_SUMMARY.md           # 日常总结
│   ├── PERFORMANCE_OPTIMIZATION.md # 性能优化文档
│   ├── TIME_SYNC_FIX.md           # 时间同步修复
│   ├── TIME_STAMP_JUMP_ISSUE.md   # 时间戳问题
│   ├── IMU_DATA_LOSS_ISSUE.md     # IMU问题
│   └── patches/                   # 补丁文件
│       └── fix_livox_cmake.patch  # CMake修复补丁
├── fastlivo2_ws/                  # FAST-LIVO2主工作空间
├── livox_ws/                      # Livox激光雷达驱动工作空间
├── sync_livox_ws/                 # 同步Livox工作空间
├── Livox-SDK2/                   # Livox SDK源码
└── backups/                       # 备份文件
```

## 快速开始

### 在新电脑上部署

1. **安装ROS2 Humble**
   ```bash
   sudo apt update
   sudo apt install ros-humble-desktop python3-colcon-common-extensions
   ```

2. **安装依赖**
   ```bash
   # Sophus（必需）
   sudo apt install ros-humble-sophus
   
   # 其他依赖
   sudo apt install libpcl-dev libeigen3-dev libopencv-dev
   ```

3. **编译工作空间**
   ```bash
   source /opt/ros/humble/setup.bash
   ./build_all_workspaces.sh
   ```

4. **运行系统**
   ```bash
   source fastlivo2_ws/install/setup.bash
   ros2 launch fast_livo mapping_mid360.launch.py use_rviz:=True
   ```

## 详细文档

所有文档都在 `docs/` 目录下：

- **[BUILD_WORKSPACES.md](docs/BUILD_WORKSPACES.md)** - 完整的编译和部署指南
- **[README_FAST_LIVO.md](docs/README_FAST_LIVO.md)** - FAST-LIVO2项目详细说明
- **[docs/README.md](docs/README.md)** - 文档目录索引

## 工作空间说明

### 1. livox_ws
Livox激光雷达驱动工作空间，提供 `livox_ros_driver2` 包。

### 2. sync_livox_ws
同步Livox工作空间，用于多激光雷达同步场景。

### 3. fastlivo2_ws
FAST-LIVO2主工作空间，包含LiDAR-Inertial-Visual融合SLAM算法。

## 依赖关系

```
fastlivo2_ws
  ├── 依赖 livox_ros_driver2 (来自 livox_ws)
  ├── 依赖 rpg_vikit
  └── 依赖 Sophus

livox_ws
  └── 提供 livox_ros_driver2

sync_livox_ws
  └── 提供 livox_ros_driver2 (独立实例)
```

## 技术栈

- **操作系统**: Ubuntu 22.04
- **ROS版本**: ROS2 Humble
- **编程语言**: C++14
- **主要依赖**:
  - PCL (Point Cloud Library)
  - Eigen3
  - OpenCV
  - Sophus (李群李代数库)
  - rpg_vikit (相机模型和数学工具)

## 硬件要求

- 激光雷达: Livox Avia / Mid360
- IMU: 支持多种IMU
- 相机: RealSense D435i (可选)
- CPU: 建议多核处理器
- 内存: 建议8GB以上

## 常见问题

查看 [docs/BUILD_WORKSPACES.md](docs/BUILD_WORKSPACES.md) 的常见问题部分，或参考技术文档目录。

## 联系方式

- FAST-LIVO2原作者: Chunran Zheng <zhengcr@connect.hku.hk>
- 项目主页: https://github.com/Robotic-Developer-Road/FAST-LIVO2

## 许可证

本源代码遵循 GPLv2 许可证。商业用途请联系原作者。

---

## 文档更新日志

- 2026-01-09: 整理文档结构，创建统一文档目录
- 2026-01-08: 创建自动编译脚本和编译指南
- 2026-01-04: 初始项目结构
