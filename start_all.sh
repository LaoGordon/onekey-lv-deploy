#!/bin/bash

###############################################################################
# FAST-LIVO 强制启动脚本
# 特性：
# 1. 移除"进程检测"，强制打开所有窗口（解决看不到终端的问题）
# 2. 窗口执行完后保留 Shell，绝对不闪退
# 3. 包含 RealSense 和日志保存
###############################################################################

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
LOG_DIR="$SCRIPT_DIR/logs"
mkdir -p "$LOG_DIR"

echo "=========================================="
echo "  FAST-LIVO 强制启动模式"
echo "=========================================="

# ==============================================================================
# 1. 生成 Livox 启动脚本
# ==============================================================================
cat > "$SCRIPT_DIR/run_livox.sh" << EOF
#!/bin/bash
echo -e "\033[32m[启动] Livox Driver...\033[0m"
cd "$SCRIPT_DIR/livox_ws"
source install/setup.bash
# 运行指令
ros2 launch livox_ros_driver2 msg_MID360_launch.py user_config_path:="$SCRIPT_DIR/livox_ws/src/livox_ros_driver2/config/MID360_config.json" 2>&1 | tee "$LOG_DIR/livox_driver.log"
echo "------------------------------------------"
echo "❌ 程序已退出/报错。窗口保留中..."
echo "你可以向上滚动查看报错信息。"
# 关键：切换回 bash shell，确保窗口不关
exec bash
EOF
chmod +x "$SCRIPT_DIR/run_livox.sh"

# ==============================================================================
# 2. 生成 RealSense 启动脚本
# ==============================================================================
cat > "$SCRIPT_DIR/run_realsense.sh" << EOF
#!/bin/bash
echo -e "\033[32m[启动] RealSense Camera...\033[0m"
cd "$SCRIPT_DIR"
source /opt/ros/humble/setup.bash
# 运行指令
ros2 launch realsense2_camera rs_launch.py rgb_camera.profile:=640x480x30 align_depth.enable:=false 2>&1 | tee "$LOG_DIR/realsense.log"
echo "------------------------------------------"
echo "❌ 程序已退出/报错。窗口保留中..."
exec bash
EOF
chmod +x "$SCRIPT_DIR/run_realsense.sh"

# ==============================================================================
# 3. 生成 FAST-LIVO 启动脚本 (已修正延时)
# ==============================================================================
cat > "$SCRIPT_DIR/run_livo.sh" << EOF
#!/bin/bash
echo -e "\033[33m[等待] 正在等待雷达时间同步 (5秒)..."
echo "请不要关闭窗口，耐心等待..."
# 这里改成 5 秒，确保雷达驱动完全启动并同步时间
sleep 5

echo -e "\033[32m[启动] FAST-LIVO...\033[0m"
cd "$SCRIPT_DIR"
source /opt/ros/humble/setup.bash
source "$SCRIPT_DIR/livox_ws/install/setup.bash"
source "$SCRIPT_DIR/fastlivo2_ws/install/setup.bash"

# 运行指令
ros2 launch fast_livo mapping_mid360.launch.py 2>&1 | tee "$LOG_DIR/fastlivo.log"

echo "------------------------------------------"
echo "❌ 程序已退出/报错。窗口保留中..."
exec bash
EOF
chmod +x "$SCRIPT_DIR/run_livo.sh"

# ==============================================================================
# 4. 强制打开所有标签页
# ==============================================================================
echo "正在打开新终端窗口..."

# 不再做 check_process_running 判断，直接添加所有命令
gnome-terminal --window \
    --tab --title="Livox Driver" --command="bash '$SCRIPT_DIR/run_livox.sh'" \
    --tab --title="RealSense"    --command="bash '$SCRIPT_DIR/run_realsense.sh'" \
    --tab --title="FAST-LIVO"    --command="bash '$SCRIPT_DIR/run_livo.sh'"

echo ""
echo "✅ 已发送启动命令。"
echo "⚠️  请查看【新弹出的终端窗口】，里面有三个标签页。"
