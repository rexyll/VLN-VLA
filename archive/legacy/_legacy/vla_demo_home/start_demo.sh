#!/bin/bash
# start_demo.sh —— 启动完整演示 (Gazebo GUI 显示在虚拟机桌面 :0)
# 项目根目录: $VLA_HOME (默认 /home/earl/workspace/VLA, 即 Windows 端 Z:\VLA)
export VLA_HOME="${VLA_HOME:-/home/earl/workspace/VLA}"
source /opt/ros/jazzy/setup.bash
[ -f "$VLA_HOME/vla_ws/install/setup.bash" ] && source "$VLA_HOME/vla_ws/install/setup.bash"

echo ">>> 清理旧进程..."
pkill -f 'ros2 launch vla_follow' 2>/dev/null
pkill -f 'vla_follow/lib/vla_follow' 2>/dev/null
pkill -f parameter_bridge 2>/dev/null
pkill -f 'gz sim' 2>/dev/null
pkill -f 'gz-sim' 2>/dev/null
sleep 2
pkill -9 -f 'gz sim' 2>/dev/null
pkill -9 -f 'vla_follow/lib/vla_follow' 2>/dev/null
pkill -9 -f parameter_bridge 2>/dev/null
sleep 1

export DISPLAY=:0
mkdir -p "$VLA_HOME/vla_ws/logs"
setsid nohup ros2 launch vla_follow demo.launch.py "$@" \
    > "$VLA_HOME/vla_ws/logs/demo.log" 2>&1 < /dev/null &
echo ">>> DEMO 已启动 (pid=$!), 日志: $VLA_HOME/vla_ws/logs/demo.log"
sleep 5
# GUI 切换为全局俯视视角(场地中心正上方, 12m 工作区全覆盖)
gz topic -t /gui/camera/pose -m gz.msgs.Pose -p \
  'position: {x: 0.001, y: -0.001, z: 16} orientation: {w: 0.711, x: 0.0, y: -0.704, z: 0.0}' \
  >/dev/null 2>&1
sleep 3

# 日志终端: 顶部窄条, 不遮挡主视图(全局俯视)与底部控件
export DBUS_SESSION_BUS_ADDRESS="unix:path=/run/user/$(id -u)/bus"
nohup gnome-terminal --geometry=100x11+140+34 -- bash -c \
    "tail -n 8 -F $VLA_HOME/vla_ws/logs/demo.log; exec bash" \
    >/dev/null 2>&1
sleep 1
wmctrl -r "Gazebo Sim" -b add,maximized_vert,maximized_horz 2>/dev/null
echo ">>> 桌面布局: Gazebo 最大化(全局俯视), 日志终端在顶部"
