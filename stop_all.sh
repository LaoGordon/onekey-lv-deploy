#!/bin/bash

# FAST-LIVO 一键停止脚本
# 通过 PID 文件停止所有后台节点

echo "正在停止所有 FAST-LIVO 节点..."

# 获取脚本所在目录
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
LOG_DIR="$SCRIPT_DIR/logs"

# 停止 Livox 驱动
if [ -f "$LOG_DIR/livox_driver.pid" ]; then
    LIVOX_PID=$(cat "$LOG_DIR/livox_driver.pid")
    if ps -p $LIVOX_PID > /dev/null 2>&1; then
        kill $LIVOX_PID
        echo "已停止 Livox 驱动 (PID: $LIVOX_PID)"
    else
        echo "Livox 驱动进程 (PID: $LIVOX_PID) 不存在"
    fi
    rm -f "$LOG_DIR/livox_driver.pid"
else
    echo "未找到 Livox 驱动 PID 文件"
fi

# 停止 RealSense 相机
if [ -f "$LOG_DIR/realsense.pid" ]; then
    REALSENSE_PID=$(cat "$LOG_DIR/realsense.pid")
    if ps -p $REALSENSE_PID > /dev/null 2>&1; then
        kill $REALSENSE_PID
        echo "已停止 RealSense 相机 (PID: $REALSENSE_PID)"
    else
        echo "RealSense 相机进程 (PID: $REALSENSE_PID) 不存在"
    fi
    rm -f "$LOG_DIR/realsense.pid"
else
    echo "未找到 RealSense 相机 PID 文件"
fi

# 停止 FAST-LIVO
if [ -f "$LOG_DIR/fastlivo.pid" ]; then
    FASTLIVO_PID=$(cat "$LOG_DIR/fastlivo.pid")
    if ps -p $FASTLIVO_PID > /dev/null 2>&1; then
        kill $FASTLIVO_PID
        echo "已停止 FAST-LIVO (PID: $FASTLIVO_PID)"
    else
        echo "FAST-LIVO 进程 (PID: $FASTLIVO_PID) 不存在"
    fi
    rm -f "$LOG_DIR/fastlivo.pid"
else
    echo "未找到 FAST-LIVO PID 文件"
fi

# 等待2秒
sleep 2

# 检查是否还有残留进程
REMAINING=$(ps aux | grep -E "livox_ros_driver2|realsense2_camera|fastlivo_mapping" | grep -v grep)

if [ -n "$REMAINING" ]; then
    echo "警告：仍有残留进程，正在强制停止..."
    pkill -9 -f "livox_ros_driver2"
    pkill -9 -f "realsense2_camera"
    pkill -9 -f "fastlivo_mapping"
    sleep 1
fi

echo "所有节点已停止！"
echo "日志文件保存在: $LOG_DIR"
