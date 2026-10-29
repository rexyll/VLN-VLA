#!/usr/bin/env bash
# run_gazebo.sh —— 启动 VLA 仿真环境（headless：虚拟显示器 + 软件渲染）
# 用法: bash run_gazebo.sh          前台运行（Ctrl+C 退出）
#       nohup bash run_gazebo.sh &  后台运行，日志见 gzserver.log
set -e
cd "$(dirname "$0")/gazebo"
pkill -f "gz sim" 2>/dev/null || true
pkill -f gzserver 2>/dev/null || true
sleep 1
export LIBGL_ALWAYS_SOFTWARE=1          # 虚拟机无 GPU：软件渲染
export GZ_SIM_SYSTEM_PLUGIN_PATH="$(pwd):$GZ_SIM_SYSTEM_PLUGIN_PATH"
echo "[run_gazebo] starting gzserver (world=vla_arena) ..."
exec gz sim -s -r --verbose 1 vla_arena.world
