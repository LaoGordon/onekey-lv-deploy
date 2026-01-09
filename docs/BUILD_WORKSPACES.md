# LV工作空间自动编译脚本使用说明

## 概述

`build_all_workspaces.sh` 是一个自动化脚本，用于在新电脑上编译LV项目的所有ROS2工作空间。

## 脚本功能

- 自动检测ROS2环境
- 自动检查并安装Sophus依赖
- 自动编译和安装Livox-SDK2
- 清理旧的编译文件
- 依次编译三个工作空间（按依赖顺序）：
  1. `livox_ws` - Livox激光雷达驱动工作空间（基础依赖）
  2. `sync_livox_ws` - 同步Livox工作空间
  3. `fastlivo2_ws` - FAST-LIVO2主工作空间（依赖livox_ros_driver2）
- 显示编译进度和结果
- 统计编译时间

## 使用步骤

### 1. 准备工作

在新电脑上，确保已经安装：

```bash
# Ubuntu 22.04 + ROS2 Humble
sudo apt update
sudo apt install ros-humble-desktop python3-colcon-common-extensions
```

### 2. 安装FAST-LIVO2依赖

**重要**: FAST-LIVO2需要Sophus库，脚本会自动检查和安装：

```bash
# 如果未安装，脚本会自动执行:
# sudo apt install ros-humble-sophus
```

如果需要手动安装：
```bash
# 方法1: 通过apt安装（推荐）
sudo apt install ros-humble-sophus

# 方法2: 如果apt版本不兼容，手动编译安装
git clone https://github.com/strasdat/Sophus.git
cd Sophus
git checkout a621ff
mkdir build && cd build && cmake ..
make
sudo make install
```

### 3. 安装其他依赖

```bash
# PCL、Eigen、OpenCV（通常ROS2桌面版已包含）
sudo apt install libpcl-dev libeigen3-dev libopencv-dev

# Python依赖
pip install rosbags
```

### 4. 移动LV目录

将整个`LV`目录移动到新电脑的目标位置，例如：

```bash
# 通过U盘或网络传输
# 移动到 ~/LV 或其他位置
```

### 5. Source ROS2环境

```bash
source /opt/ros/humble/setup.bash
```

### 6. 运行编译脚本

```bash
cd ~/LV  # 进入LV目录
./build_all_workspaces.sh
```

### 7. 验证编译结果

脚本完成后，会显示编译总结。如果所有工作空间编译成功，可以运行：

```bash
source fastlivo2_ws/install/setup.bash
ros2 launch fast_livo mapping_mid360.launch.py use_rviz:=True
```

## 脚本输出示例

```
========================================
  LV工作空间自动编译脚本
========================================

[INFO] 检查必要命令...
[SUCCESS] colcon 已安装
[SUCCESS] cmake 已安装

[INFO] 检查ROS2环境...
[SUCCESS] ROS2环境已设置: humble

[INFO] 检查Sophus依赖...
[SUCCESS] Sophus已安装: ros-humble-sophus

[INFO] ========================================
[INFO] 编译和安装Livox-SDK2
[INFO] ========================================
[INFO] 清理旧的构建文件...
[INFO] 开始编译Livox-SDK2...
[SUCCESS] CMake配置成功
[SUCCESS] Livox-SDK2编译成功
[INFO] 安装Livox-SDK2...
[SUCCESS] Livox-SDK2安装成功!

[INFO] ========================================
[INFO] 开始编译工作空间: livox_ws
[INFO] 路径: livox_ws
[INFO] ========================================
[INFO] 清理旧的构建文件...
[INFO] 开始编译...
[SUCCESS] 工作空间 livox_ws 编译成功!

... (继续编译其他工作空间)

========================================
  编译总结
========================================
总工作空间数: 3
编译成功: 3
编译失败: 0
总耗时: 5分 23秒

[SUCCESS] 所有工作空间编译完成!

[INFO] 使用方法:
  source fastlivo2_ws/install/setup.bash
  ros2 launch fast_livo mapping_mid360.launch.py use_rviz:=True
```

## 注意事项

### 编译前检查

1. **ROS2环境**：确保已source ROS2环境
2. **Sophus依赖**：脚本会自动检查并安装，但确保网络连接正常
3. **Livox-SDK2**：脚本会自动编译安装，确保LV/Livox-SDK2目录存在
4. **磁盘空间**：确保有足够的磁盘空间（至少5GB）
5. **网络连接**：首次编译可能需要下载依赖

### 依赖安装清单

#### 必需依赖

```bash
# ROS2基础
sudo apt install ros-humble-desktop python3-colcon-common-extensions

# Sophus（脚本会自动检查和安装）
# 如果未安装，脚本会自动执行: sudo apt install ros-humble-sophus

# Livox SDK（脚本会自动编译和安装）
# 脚本会在编译工作空间前自动执行:
#   cd Livox-SDK2/build
#   cmake ..
#   make -j$(nproc)
#   sudo make install
```

#### 可选依赖

```bash
# PCL、Eigen、OpenCV（通常ROS2桌面版已包含）
sudo apt install libpcl-dev libeigen3-dev libopencv-dev

# Python工具
pip install rosbags
```

#### 如果Sophus apt版本不兼容

```bash
git clone https://github.com/strasdat/Sophus.git
cd Sophus
git checkout a621ff
mkdir build && cd build && cmake ..
make
sudo make install

# 如果遇到编译错误，修改so2.cpp:
# unit_complex_.real() = 1.;  改为  unit_complex_.real(1.);
# unit_complex_.imag() = 0.;  改为  unit_complex_.imag(0.);
```

### 常见问题

**问题1**: `colcon: command not found`
```bash
sudo apt install python3-colcon-common-extensions
```

**问题2**: ROS2环境未设置
```bash
source /opt/ros/humble/setup.bash
```

**问题3**: `Could not find Sophus`
```bash
# 检查是否安装了Sophus
dpkg -l | grep sophus

# 如果没有，安装它
sudo apt install ros-humble-sophus
```

**问题4**: `Livox-SDK2目录不存在`
```bash
# 确保LV目录完整，包含Livox-SDK2子目录
ls LV/Livox-SDK2
```

**问题5**: 编译某个工作空间失败
- 查看错误信息
- 检查是否缺少依赖（特别是Sophus）
- 尝试单独编译该工作空间

## 手动编译单个工作空间

如果需要单独编译某个工作空间：

```bash
# 编译fastlivo2_ws（需要先source livox_ws）
cd fastlivo2_ws
source ../livox_ws/install/setup.bash
colcon build --symlink-install --continue-on-error

# 编译livox_ws
cd ../livox_ws
colcon build --symlink-install --continue-on-error

# 编译sync_livox_ws
cd ../sync_livox_ws
colcon build --symlink-install --continue-on-error
```

## 工作空间编译顺序说明

**重要**: 工作空间有依赖关系，必须按以下顺序编译：

1. **livox_ws** (基础层)
   - 提供 `livox_ros_driver2` 包
   - 被 fastlivo2_ws 依赖
   - 必须最先编译

2. **sync_livox_ws** (同步层)
   - 也包含 `livox_ros_driver2`
   - 用于多激光雷达同步场景
   - 与 livox_ws 独立

3. **fastlivo2_ws** (应用层)
   - 依赖 `livox_ros_driver2`
   - 依赖 `rpg_vikit`
   - 依赖 `Sophus`
   - 必须在 livox_ws 编译成功后才能编译

## 工作空间详细说明

### 1. livox_ws
- **用途**: Livox激光雷达驱动工作空间
- **包含**: livox_ros_driver2
- **支持设备**: Livox Avia、Mid360等激光雷达
- **编译优先级**: 最高（其他工作空间的依赖）

### 2. sync_livox_ws
- **用途**: 同步Livox工作空间
- **包含**: livox_ros_driver2
- **应用场景**: 多激光雷达同步或特定同步方案
- **编译优先级**: 中等

### 3. fastlivo2_ws
- **用途**: FAST-LIVO2主工作空间
- **包含**: LiDAR-Inertial-Visual融合SLAM算法
- **依赖包**: 
  - livox_ros_driver2（来自livox_ws）
  - rpg_vikit（相机模型和数学工具）
  - Sophus（李群李代数库，**脚本会自动安装**）
- **编译优先级**: 最低（依赖其他工作空间）

## 编译选项说明

脚本使用的colcon编译选项：

- `--symlink-install`: 使用符号链接安装，加快编译速度
- `--continue-on-error`: 遇到错误继续编译其他包

## 技术支持

如遇问题，请检查：

1. ROS2版本是否为Humble
2. Ubuntu版本是否为22.04
3. **Sophus是否已安装**（最重要）
4. **Livox-SDK2目录是否存在**
5. 所有依赖项是否已安装
6. 编译错误信息中的具体提示

## 更新日志

- 2026-01-09: 更新依赖说明，添加Sophus和Livox-SDK2自动安装功能
- 2026-01-08: 初始版本，支持自动编译三个工作空间
