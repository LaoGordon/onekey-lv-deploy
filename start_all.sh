#!/bin/bash

# FAST-LIVO 一键启动脚本
# 在后台启动所有节点，输出重定向到日志文件

echo "正在启动 FAST-LIVO 系统..."

# 获取脚本所在目录
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# 创建日志目录
LOG_DIR="$SCRIPT_DIR/logs"
mkdir -p "$LOG_DIR"

# 清空旧日志（可选）
# rm -f "$LOG_DIR"/*.log

echo "日志文件保存在: $LOG_DIR"

# 1. 后台启动 Livox 驱动
echo "启动 Livox 驱动..."
cd "$SCRIPT_DIR/livox_ws" && source install/setup.bash
nohup ros2 launch livox_ros_driver2 msg_MID360_launch.py > "$LOG_DIR/livox_driver.log" 2>&1 &
LIVOX_PID=$!
echo "Livox 驱动 PID: $LIVOX_PID"
echo "$LIVOX_PID" > "$LOG_DIR/livox_driver.pid"
sleep 3

# 2. 后台启动 RealSense 相机
echo "启动 RealSense 相机..."
cd "$SCRIPT_DIR" && source /opt/ros/humble/setup.bash
nohup ros2 launch realsense2_camera rs_launch.py rgb_camera.profile:=640x480x30 align_depth.enable:=false > "$LOG_DIR/realsense.log" 2>&1 &
REALSENSE_PID=$!
echo "RealSense 相机 PID: $REALSENSE_PID"
echo "$REALSENSE_PID" > "$LOG_DIR/realsense.pid"
sleep 3

# 3. 后台启动 FAST-LIVO
echo "启动 FAST-LIVO..."
cd "$SCRIPT_DIR" && source /opt/ros/humble/setup.bash && source "$SCRIPT_DIR/livox_ws/install/setup.bash" && source "$SCRIPT_DIR/fastlivo2_ws/install/setup.bash"
nohup ros2 launch fast_livo mapping_mid360.launch.py > "$LOG_DIR/fastlivo.log" 2>&1 &
FASTLIVO_PID=$!
echo "FAST-LIVO PID: $FASTLIVO_PID"
echo "$FASTLIVO_PID" > "$LOG_DIR/fastlivo.pid"
sleep 2

echo ""
echo "=================================="
echo "所有节点已在后台启动！"
echo "=================================="
echo "Livox 驱动   PID: $LIVOX_PID   日志: $LOG_DIR/livox_driver.log"
echo "RealSense 相机 PID: $REALSENSE_PID   日志: $LOG_DIR/realsense.log"
echo "FAST-LIVO    PID: $FASTLIVO_PID   日志: $LOG_DIR/fastlivo.log"
echo "=================================="
echo ""
echo "查看日志示例："
echo "  tail -f $LOG_DIR/livox_driver.log"
echo "  tail -f $LOG_DIR/realsense.log"
echo "  tail -f $LOG_DIR/fastlivo.log"
echo ""
echo "停止所有节点: ./stop_all.sh"
