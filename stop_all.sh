#!/bin/bash

###############################################################################
# FAST-LIVO 一键停止脚本
# 直接通过进程名停止所有节点
###############################################################################

echo "=========================================="
echo "  停止 FAST-LIVO 系统"
echo "=========================================="
echo ""

# 获取脚本所在目录
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
LOG_DIR="$SCRIPT_DIR/logs"

# 停止 Livox 驱动
echo "正在停止 Livox 驱动..."
pkill -f "livox_ros_driver2" 2>/dev/null
if [ $? -eq 0 ]; then
    echo "✓ Livox 驱动已停止"
else
    echo "  Livox 驱动未运行"
fi

# 停止 RealSense 相机
echo "正在停止 RealSense 相机..."
pkill -f "realsense2_camera" 2>/dev/null
if [ $? -eq 0 ]; then
    echo "✓ RealSense 相机已停止"
else
    echo "  RealSense 相机未运行"
fi

# 停止 FAST-LIVO
echo "正在停止 FAST-LIVO..."
pkill -f "fastlivo_mapping" 2>/dev/null
if [ $? -eq 0 ]; then
    echo "✓ FAST-LIVO 已停止"
else
    echo "  FAST-LIVO 未运行"
fi

# 等待2秒，让进程优雅退出
echo ""
echo "等待进程优雅退出..."
sleep 2

# 检查是否还有残留进程，如果有则强制停止
REMAINING_LIVOX=$(pgrep -f "livox_ros_driver2" 2>/dev/null)
REMAINING_RS=$(pgrep -f "realsense2_camera" 2>/dev/null)
REMAINING_FL=$(pgrep -f "fastlivo_mapping" 2>/dev/null)

if [ -n "$REMAINING_LIVOX" ] || [ -n "$REMAINING_RS" ] || [ -n "$REMAINING_FL" ]; then
    echo ""
    echo "警告：仍有残留进程，正在强制停止..."
    pkill -9 -f "livox_ros_driver2" 2>/dev/null
    pkill -9 -f "realsense2_camera" 2>/dev/null
    pkill -9 -f "fastlivo_mapping" 2>/dev/null
    sleep 1
fi

echo ""
echo "=========================================="
echo "✓ 所有节点已停止！"
echo "=========================================="
echo "日志文件保存在: $LOG_DIR"
echo ""
