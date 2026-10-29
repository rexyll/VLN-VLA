#!/bin/bash
# start_demo.sh —— 启动完整演示 (Gazebo GUI 显示在虚拟机桌面 :0)
# 项目根目录: $VLA_HOME (默认 /home/earl/workspace/VLA, 即 Windows 端 Z:\VLA)
#
# P71 重写要点:
#   1) Nav2 节点必须按"完整路径前缀"杀(pkill -f '/opt/ros/jazzy/lib/nav2_'),
#      不能用 pkill -x 名字 —— Linux 的 comm 只有 15 字符,
#      velocity_smoother/controller_server/lifecycle_manager 会被截断,
#      按全名一个都杀不掉, 上一轮的 Nav2 就成了孤儿进程与新一轮同名共存,
#      lifecycle 的服务调用打到旧实例上 → 随机的
#      "get_state service is not available" / "Aborting bringup"。
#   2) gz 只由本脚本启动; ros2 launch 传 start_gz:=false, 不再起第二个实例。
#   3) 互斥锁做"陈旧锁自愈": 若 flock 失败但其实没有 demo 在跑, 说明上一轮
#      异常退出留下了锁文件, 删掉重来, 不必手动清场。
export VLA_HOME="${VLA_HOME:-/home/earl/workspace/VLA}"
LOCK=/tmp/vla_start_demo.lock

# P66: 实例互斥锁 —— 防止两次 start_demo 重叠执行造成双实例污染
exec 9>"$LOCK"
if ! flock -n 9; then
    if pgrep -f 'ros2 launch vla_follow' >/dev/null 2>&1; then
        echo ">>> [互斥] 另一个 start_demo 正在运行, 本次启动取消"
        exit 1
    fi
    echo ">>> [互斥] 检测到陈旧锁(demo 并未运行), 自动清除后继续"
    exec 9>&-
    rm -f "$LOCK"
    exec 9>"$LOCK"
    flock -n 9 || { echo ">>> [互斥] 仍无法加锁, 退出"; exit 1; }
fi

source /opt/ros/jazzy/setup.bash
[ -f "$VLA_HOME/vla_ws/install/setup.bash" ] && source "$VLA_HOME/vla_ws/install/setup.bash"

echo ">>> 清理旧进程..."
pkill -f 'ros2 launch vla_follow' 2>/dev/null
pkill -f 'vla_follow/lib/vla_follow' 2>/dev/null
pkill -f parameter_bridge 2>/dev/null
pkill -f '/opt/ros/jazzy/lib/nav2_' 2>/dev/null
pkill -f 'gz sim' 2>/dev/null
pkill -f 'gz-sim' 2>/dev/null
sleep 2
pkill -9 -f 'gz sim' 2>/dev/null
pkill -9 -f 'gz-sim' 2>/dev/null
pkill -9 -f '/opt/ros/jazzy/lib/nav2_' 2>/dev/null
pkill -9 -f 'vla_follow/lib/vla_follow' 2>/dev/null
pkill -9 -f parameter_bridge 2>/dev/null
sleep 1
left=$(ps -eo args --no-headers | grep -E 'gz sim|gz-sim|/opt/ros/jazzy/lib/nav2_|vla_follow/lib/vla_follow|parameter_bridge' | grep -v grep | wc -l)
echo ">>> 清理完成, 残留进程数: ${left}"

export DISPLAY=:0
mkdir -p "$VLA_HOME/vla_ws/logs"

# P65: 生成 Nav2 静态地图(已知障碍几何 → arena.pgm/yaml)
python3 "$VLA_HOME/vla_ws/src/vla_follow/vla_follow/gen_map.py" \
    "$VLA_HOME/vla_demo/maps" || echo "WARN: 地图生成失败, Nav2 将不可用"

# P76: gz 退出时会把"实时布局+相机"写回 ~/.gz/sim/8/gui.config, 会覆盖掉
# 手工配置。所以每次启动 gz 之前强制重写一次: 全场俯视 + 去掉右侧
# EntityTree/ComponentInspector 两块面板(它们吃掉一半宽度, 12m 场地放不下)。
python3 "$VLA_HOME/vla_demo/gz_gui_fix.py" || echo "WARN: GUI 配置修正失败"

# P86: gz 系统插件(抓取附着 GraspCarry)按需构建 + 设好插件搜索路径
source "$VLA_HOME/tools/gz_plugin.sh"

# P86: 生成物同步 —— tools/gen_drink_scene.py 写的是 src, 这里保证 install
# 是最新的。少了这一步会拿上一轮的 SDF 起世界(症状: 改了生成器但世界没变)。
for pair in "worlds/work_area.sdf:worlds/work_area.sdf" \
            "config/bridge.yml:config/bridge.yml"; do
    rel="${pair%%:*}"
    src="$VLA_HOME/vla_ws/src/vla_follow/$rel"
    dst="$VLA_HOME/vla_ws/install/vla_follow/share/vla_follow/$rel"
    if [ ! -L "$dst" ] && [ -f "$src" ] && [ "$src" -nt "$dst" ]; then
        cp "$src" "$dst" && echo ">>> 同步 $rel -> install"
    fi
done

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

# ROS2: 桥接 + Nav2 + 业务节点(单实例)。gz 已由上面启动 → start_gz:=false
setsid nohup ros2 launch vla_follow demo.launch.py start_gz:=false "$@" \
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
