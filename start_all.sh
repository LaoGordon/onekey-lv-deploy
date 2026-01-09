#!/bin/bash

###############################################################################
# FAST-LIVO 一键启动脚本
# 在3个终端窗口中分别启动所有节点，输出同时保存到日志
###############################################################################

# 获取脚本所在目录
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# 创建日志目录
LOG_DIR="$SCRIPT_DIR/logs"
mkdir -p "$LOG_DIR"

echo "=========================================="
echo "  FAST-LIVO 系统启动"
echo "=========================================="
echo ""

# 检测终端类型
if command -v gnome-terminal &> /dev/null; then
    TERMINAL="gnome-terminal"
elif command -v xterm &> /dev/null; then
    TERMINAL="xterm"
elif command -v konsole &> /dev/null; then
    TERMINAL="konsole"
else
    echo "错误：未找到可用的终端"
    echo "请安装 gnome-terminal、xterm 或 konsole"
    exit 1
fi

echo "使用终端: $TERMINAL"
echo "日志目录: $LOG_DIR"
echo ""

# 启动 Livox 驱动
echo "启动 Livox 驱动..."
$TERMINAL --title="Livox Driver" -- bash -c "
    cd '$SCRIPT_DIR/livox_ws' && source install/setup.bash
    ros2 launch livox_ros_driver2 msg_MID360_launch.py 2>&1 | tee '$LOG_DIR/livox_driver.log'
    read
" &
sleep 1

# 启动 RealSense 相机
echo "启动 RealSense 相机..."
$TERMINAL --title="RealSense Camera" -- bash -c "
    cd '$SCRIPT_DIR' && source /opt/ros/humble/setup.bash
    ros2 launch realsense2_camera rs_launch.py rgb_camera.profile:=640x480x30 align_depth.enable:=false 2>&1 | tee '$LOG_DIR/realsense.log'
    read
" &
sleep 1

# 启动 FAST-LIVO
echo "启动 FAST-LIVO..."
$TERMINAL --title="FAST-LIVO" -- bash -c "
    cd '$SCRIPT_DIR' && source /opt/ros/humble/setup.bash && source '$SCRIPT_DIR/livox_ws/install/setup.bash' && source '$SCRIPT_DIR/fastlivo2_ws/install/setup.bash'
    ros2 launch fast_livo mapping_mid360.launch.py 2>&1 | tee '$LOG_DIR/fastlivo.log'
    read
" &
sleep 1

echo ""
echo "=========================================="
echo "已在3个终端窗口启动所有节点！"
echo "=========================================="
echo "Livox Driver   - 终端窗口   日志: $LOG_DIR/livox_driver.log"
echo "RealSense Camera - 终端窗口   日志: $LOG_DIR/realsense.log"
echo "FAST-LIVO      - 终端窗口   日志: $LOG_DIR/fastlivo.log"
echo "=========================================="
echo ""
echo "停止所有节点: ./stop_all.sh"
