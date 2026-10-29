#!/bin/bash
# show_follow.sh —— 一键演示：Gazebo(带实时轨迹) + 四相机跟随
#
# 用法:
#   ./show_follow.sh              # 默认跟随距离 1.0m
#   ./show_follow.sh 1.5          # 跟随距离 1.5m
#
# 窗口里怎么看:
#   橙色小点 = 目标的行走轨迹      青色小点 = 机器人的行驶轨迹
#   品红色球 = 目标当前位置        黄色球   = 机器人当前位置
#   两端的小方块(青色/橙色) = 当前朝向
#   灰色小球 = 起点
#
# 轨迹可视化由 Gazebo GUI 自带的 MarkerManager 渲染，前置条件:
#   ~/.gz/sim/8/gui.config 里 MarkerManager 插件带 <topic_name>/marker</topic_name>
#   （本仓库的 setup 脚本已处理；缺失时 traj_viz.py 会直接报错退出）
set +u
export DISPLAY=${DISPLAY:-:0}          # SSH 启动时必须指到桌面会话, 否则 gz 的 Qt 初始化失败
VLA_HOME=${VLA_HOME:-/home/earl/workspace/VLA}
cd "$VLA_HOME" || exit 1
source /opt/ros/jazzy/setup.bash
source "$VLA_HOME/vla_ws/install/setup.bash"

DIST=${1:-1.0}
WORLD=${VLA_WORLD:-follow_world}

cleanup() {
  echo
  echo "[SHOW] 收尾: 停止轨迹可视化与仿真"
  pkill -f 'traj_vi[z].py'
  pkill -f 'ros2 launc[h]'
  pkill -f 'gz si[m]'
}
trap cleanup INT TERM

echo "[SHOW] 启动 Gazebo (world=$WORLD) ..."
ros2 launch vla_follow demo.launch.py auto_voice:=false > "$VLA_HOME/show_follow.log" 2>&1 &
LAUNCH_PID=$!
sleep 25

echo "[SHOW] 启动轨迹可视化 ..."
python3 "$VLA_HOME/vla_demo/traj_viz.py" --world "$WORLD" \
  --out "$VLA_HOME/traj_log.csv" > "$VLA_HOME/traj_viz.log" 2>&1 &
VIZ_PID=$!
sleep 3

echo "[SHOW] 发送跟随指令 (距离 ${DIST}m) ..."
ros2 topic pub --once /robot/command std_msgs/msg/String \
  "{data: '{\"action\":\"FOLLOW\",\"distance\":${DIST},\"text\":\"请跟随我\"}'}" >/dev/null 2>&1

echo
echo "[SHOW] 演示进行中。Gazebo 窗口:"
echo "        橙色点 = 目标轨迹    青色点 = 机器人轨迹"
echo "        品红球 = 目标位置    黄球   = 机器人位置"
echo "[SHOW] 轨迹 CSV: $VLA_HOME/traj_log.csv"
echo "[SHOW] 按 Ctrl-C 结束"
wait $LAUNCH_PID
