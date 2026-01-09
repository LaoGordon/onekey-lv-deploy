# LV项目文档目录

本目录包含LV项目的所有文档和补丁文件。

## 目录结构

```
docs/
├── README.md                          # 本文件 - 文档目录说明
├── BUILD_WORKSPACES.md               # 工作空间编译说明
├── README_FAST_LIVO.md               # FAST-LIVO2项目说明
├── DAILY_SUMMARY.md                  # 日常总结文档
├── PERFORMANCE_OPTIMIZATION.md      # 性能优化文档
├── TIME_SYNC_FIX.md                  # 时间同步修复文档
├── TIME_STAMP_JUMP_ISSUE.md         # 时间戳跳变问题文档
├── IMU_DATA_LOSS_ISSUE.md           # IMU数据丢失问题文档
└── patches/                          # 补丁文件目录
    └── fix_livox_cmake.patch        # livox_ros_driver2的CMakeLists.txt修复补丁
```

## 文档说明

### 编译和部署相关
- **BUILD_WORKSPACES.md**: 详细说明如何在新电脑上编译所有工作空间，包括依赖安装、编译步骤和常见问题
- **README_FAST_LIVO.md**: FAST-LIVO2项目的完整说明，包括安装、配置、运行方法

### 技术文档
- **PERFORMANCE_OPTIMIZATION.md**: 性能优化的详细记录和优化方案
- **TIME_SYNC_FIX.md**: 时间同步问题的修复过程和解决方案
- **TIME_STAMP_JUMP_ISSUE.md**: 时间戳跳变问题的分析和解决
- **IMU_DATA_LOSS_ISSUE.md**: IMU数据丢失问题的排查和修复

### 项目记录
- **DAILY_SUMMARY.md**: 日常开发工作的总结记录

### 补丁文件
- **patches/**: 包含各种修复补丁
  - `fix_livox_cmake.patch`: 修复livox_ros_driver2在ROS2 Humble下的编译问题

## 使用指南

### 新电脑部署
1. 阅读 `BUILD_WORKSPACES.md` 了解编译流程
2. 按照文档步骤安装依赖
3. 运行 `build_all_workspaces.sh` 编译所有工作空间

### 功能开发参考
1. 阅读 `README_FAST_LIVO.md` 了解项目功能
2. 参考技术文档了解特定问题的解决方案

### 问题排查
1. 查看相关技术文档
2. 检查 `patches/` 目录是否有适用的补丁

## 注意事项

- 所有文档都应该保持最新状态
- 修改代码后，请更新相关文档
- 添加新的技术文档时，请在此文件中更新目录结构
