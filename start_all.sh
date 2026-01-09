#!/bin/bash

###############################################################################
# FAST-LIVO 一键启动脚本 (数组修复版)
###############################################################################

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
LOG_DIR="$SCRIPT_DIR/logs"
mkdir -p "$LOG_DIR"

echo "=========================================="
echo "  FAST-LIVO 系统启动"
echo "=========================================="

# 检查进程是否已经在运行的函数
check_process_running() {
    local process_name=$1
    local display_name=$2
    
    # 排除 grep 自身和当前脚本
    if pgrep -f "$process_name" | grep -v $$ > /dev/null; then
        echo "  [检查] $display_name 已在运行，跳过启动"
        return 0  # 已运行
    else
        echo "  [检查] $display_name 未运行，准备启动"
        return 1  # 未运行
    fi
}

# 检查各服务状态
echo ""
echo "检查服务状态..."
LIVOX_RUNNING=false
REALSENSE_RUNNING=false
FASTLIVO_RUNNING=false

check_process_running "msg_MID360_launch" "Livox Driver" && LIVOX_RUNNING=true
check_process_running "realsense2_camera" "RealSense Camera" && REALSENSE_RUNNING=true
check_process_running "mapping_mid360" "FAST-LIVO" && FASTLIVO_RUNNING=true

echo ""

# 核心修改：使用数组构建命令
CMD_ARRAY=()

if [ "$LIVOX_RUNNING" = false ]; then
    CMD_ARRAY+=(
        --tab --title="Livox Driver" --command="bash -c '
            echo \"启动 Livox Driver...\";
            cd \"$SCRIPT_DIR/livox_ws\";
            source install/setup.bash;
            ros2 launch livox_ros_driver2 msg_MID360_launch.py 2>&1 | tee \"$LOG_DIR/livox_driver.log\";
            echo \"进程已结束，按回车退出\"; read
        '"
    )
fi

if [ "$REALSENSE_RUNNING" = false ]; then
    CMD_ARRAY+=(
        --tab --title="RealSense Camera" --command="bash -c '
            echo \"启动 RealSense...\";
            cd \"$SCRIPT_DIR\";
            source /opt/ros/humble/setup.bash;
            ros2 launch realsense2_camera rs_launch.py rgb_camera.profile:=640x480x30 align_depth.enable:=false 2>&1 | tee \"$LOG_DIR/realsense.log\";
            echo \"进程已结束，按回车退出\"; read
        '"
    )
fi

if [ "$FASTLIVO_RUNNING" = false ]; then
    CMD_ARRAY+=(
        --tab --title="FAST-LIVO" --command="bash -c '
            echo \"启动 FAST-LIVO...\";
            sleep 2; 
            cd \"$SCRIPT_DIR\";
            source /opt/ros/humble/setup.bash;
            source \"$SCRIPT_DIR/livox_ws/install/setup.bash\";
            source \"$SCRIPT_DIR/fastlivo2_ws/install/setup.bash\";
            ros2 launch fast_livo mapping_mid360.launch.py 2>&1 | tee \"$LOG_DIR/fastlivo.log\";
            echo \"进程已结束，按回车退出\"; read
        '"
    )
fi

# 执行命令
# ${#CMD_ARRAY[@]} 获取数组长度
if [ ${#CMD_ARRAY[@]} -gt 0 ]; then
    # "${CMD_ARRAY[@]}" 会将数组元素原封不动地展开，保留引号和空格
    gnome-terminal --window "${CMD_ARRAY[@]}"
    
    echo ""
    echo "=========================================="
    echo "节点启动指令已发送。"
    echo "=========================================="
    
    # 显示启动的服务状态
    if [ "$LIVOX_RUNNING" = true ]; then
        echo "  ✓ Livox Driver (已在运行)"
    else
        echo "  → Livox Driver (正在启动)"
    fi
    
    if [ "$REALSENSE_RUNNING" = true ]; then
        echo "  ✓ RealSense Camera (已在运行)"
    else
        echo "  → RealSense Camera (正在启动)"
    fi
    
    if [ "$FASTLIVO_RUNNING" = true ]; then
        echo "  ✓ FAST-LIVO (已在运行)"
    else
        echo "  → FAST-LIVO (正在启动)"
    fi
    
    echo "=========================================="
    echo "日志目录: $LOG_DIR"
    echo "停止所有节点: ./stop_all.sh"
else
    echo "所有服务已在运行，无需启动新服务"
    echo ""
    echo "=========================================="
    echo "当前运行状态："
    echo "  ✓ Livox Driver"
    echo "  ✓ RealSense Camera"
    echo "  ✓ FAST-LIVO"
    echo "=========================================="
    echo "日志目录: $LOG_DIR"
    echo "停止所有节点: ./stop_all.sh"
fi

echo ""
