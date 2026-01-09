#!/bin/bash

###############################################################################
# FAST-LIVO 一键停止脚本
# 停止 Livox, RealSense, FAST-LIVO 所有节点
###############################################################################

echo "=========================================="
echo "  停止 FAST-LIVO 系统"
echo "=========================================="
echo ""

# 获取脚本所在目录
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
LOG_DIR="$SCRIPT_DIR/logs"

# 停止 Livox 驱动
echo "正在停止 Livox Driver..."
pkill -f "msg_MID360_launch" 2>/dev/null
if [ $? -eq 0 ]; then
    echo "✓ Livox Driver 已停止"
else
    echo "  Livox Driver 未运行"
fi

# 停止 RealSense 相机
echo "正在停止 RealSense Camera..."
pkill -f "realsense2_camera" 2>/dev/null
if [ $? -eq 0 ]; then
    echo "✓ RealSense Camera 已停止"
else
    echo "  RealSense Camera 未运行"
fi

# 停止 FAST-LIVO
echo "正在停止 FAST-LIVO..."
pkill -f "mapping_mid360" 2>/dev/null
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
REMAINING_LIVOX=$(pgrep -f "msg_MID360_launch" 2>/dev/null)
REMAINING_RS=$(pgrep -f "realsense2_camera" 2>/dev/null)
REMAINING_FL=$(pgrep -f "mapping_mid360" 2>/dev/null)

if [ -n "$REMAINING_LIVOX" ] || [ -n "$REMAINING_RS" ] || [ -n "$REMAINING_FL" ]; then
    echo ""
    echo "警告：仍有残留进程，正在强制停止..."
    pkill -9 -f "msg_MID360_launch" 2>/dev/null
    pkill -9 -f "realsense2_camera" 2>/dev/null
    pkill -9 -f "mapping_mid360" 2>/dev/null
    sleep 1
fi

# 清理临时启动脚本
echo ""
echo "清理临时启动脚本..."
if [ -f "$SCRIPT_DIR/run_livox.sh" ]; then
    rm "$SCRIPT_DIR/run_livox.sh"
    echo "✓ 已删除 run_livox.sh"
fi
if [ -f "$SCRIPT_DIR/run_realsense.sh" ]; then
    rm "$SCRIPT_DIR/run_realsense.sh"
    echo "✓ 已删除 run_realsense.sh"
fi
if [ -f "$SCRIPT_DIR/run_livo.sh" ]; then
    rm "$SCRIPT_DIR/run_livo.sh"
    echo "✓ 已删除 run_livo.sh"
fi

echo ""
echo "=========================================="
echo "✓ 所有节点已停止！"
echo "=========================================="
echo "日志文件保存在: $LOG_DIR"
echo ""
