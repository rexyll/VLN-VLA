#!/bin/bash
# start_demo.sh —— 启动完整演示 (Gazebo GUI 显示在虚拟机桌面 :0)
# 项目根目录: $VLA_HOME (默认 /home/earl/workspace/VLA, 即 Windows 端 Z:\VLA)
export VLA_HOME="${VLA_HOME:-/home/earl/workspace/VLA}"

# P66: 实例互斥锁 —— 防止两次 start_demo 重叠执行造成双实例污染
exec 9>/tmp/vla_start_demo.lock
if ! flock -n 9; then
    echo ">>> [互斥] 另一个 start_demo 正在运行, 本次启动取消"
    exit 1
fi

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
echo ">>> 清理完成, 残留 gz 进程: $(pgrep -c -f 'gz sim' 2>/dev/null || echo 0)"

export DISPLAY=:0
# P65: 生成 Nav2 静态地图(已知障碍几何 → arena.pgm/yaml)
python3 "$VLA_HOME/vla_ws/src/vla_follow/vla_follow/gen_map.py" \
    "$VLA_HOME/vla_demo/maps" || echo "WARN: 地图生成失败, Nav2 将不可用"
mkdir -p "$VLA_HOME/vla_ws/logs"

# 单进程 gz sim(server+GUI); 注意不要关闭 Gazebo 窗口(会终止仿真)
setsid nohup gz sim -r -v 3 --render-engine ogre \
    "$VLA_HOME/vla_ws/install/vla_follow/share/vla_follow/worlds/work_area.sdf" \
    > "$VLA_HOME/vla_ws/logs/gzsim.log" 2>&1 < /dev/null &
echo ">>> gz sim 启动中..."
for i in $(seq 1 40); do
  if timeout 3 gz topic -l 2>/dev/null | grep -q '/world/follow_world/dynamic_pose/info'; then
    echo ">>> gz topics 出现 (${i}s)"
    break
  fi
  sleep 1
done
sleep 5

# ROS2: 桥接 + Nav2 + 业务节点(单实例)
setsid nohup ros2 launch vla_follow demo.launch.py "$@" \
    > "$VLA_HOME/vla_ws/logs/demo.log" 2>&1 < /dev/null &
LAUNCH_PID=$!
echo ">>> DEMO 已启动 (pid=$LAUNCH_PID), 日志: $VLA_HOME/vla_ws/logs/demo.log"
sleep 8

# GUI 切换为全局俯视视角(场地中心正上方, 12m 工作区全覆盖)
gz topic -t /gui/camera/pose -m gz.msgs.Pose -p \
  'position: {x: 0.001, y: -0.001, z: 16} orientation: {w: 0.711, x: 0.0, y: -0.704, z: 0.0}' \
  >/dev/null 2>&1
sleep 2

# 日志终端: 顶部窄条, 不遮挡主视图
export DBUS_SESSION_BUS_ADDRESS="unix:path=/run/user/$(id -u)/bus"
nohup gnome-terminal --geometry=100x11+140+34 -- bash -c \
    "tail -n 8 -F $VLA_HOME/vla_ws/logs/demo.log; exec bash" \
    >/dev/null 2>&1
sleep 1
wmctrl -r "Gazebo Sim" -b add,maximized_vert,maximized_horz 2>/dev/null
echo ">>> 单实例校验: ros2 launch=$(pgrep -c -f 'ros2 launch vla_follow'), gz sim=$(pgrep -c -f 'gz sim')"
echo ">>> 启动完成"
