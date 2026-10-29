#!/bin/bash
# start_scene.sh —— 只起"世界 + 桥接"(不起 Nav2/业务节点), 用于场景与模型验收
#
# 用途: P84 这类"只要看模型/通道对不对"的验证, 起全套 demo 太重(还会被人
# 随机行走、Nav2 生命周期干扰)。本脚本只做三件事:
#   1) 清场
#   2) gz sim 服务端(-s, 无 GUI) —— 注意必须 export DISPLAY, 否则
#      sensors-system 的渲染线程 "Unable to open display" 直接 abort
#   3) ros_gz_bridge (bridge.yml 全量)
#
# 用法: bash tools/start_scene.sh [--gui]
# 注意: 不能开 set -u —— /opt/ros/jazzy/setup.bash 会引用未定义的
#       AMENT_TRACE_SETUP_FILES 直接报错退出。
export VLA_HOME="${VLA_HOME:-/home/earl/workspace/VLA}"
export DISPLAY="${DISPLAY:-:0}"
WITH_GUI=0
[ "${1:-}" = "--gui" ] && WITH_GUI=1

source /opt/ros/jazzy/setup.bash
source "$VLA_HOME/vla_ws/install/setup.bash"

bash "$VLA_HOME/tools/killall_demo.sh"
mkdir -p "$VLA_HOME/vla_ws/logs"

# ---- gz 系统插件(抓取附着 GraspCarry): 按需构建 + 设好插件搜索路径
source "$VLA_HOME/tools/gz_plugin.sh"

WORLD="$VLA_HOME/vla_ws/install/vla_follow/share/vla_follow/worlds/work_area.sdf"
BRIDGE="$VLA_HOME/vla_ws/install/vla_follow/share/vla_follow/config/bridge.yml"

# ---- 生成物同步: tools/gen_drink_scene.py 写的是 src, 这里保证 install 是最新的
#      (少了这一步会拿上一轮的 SDF 起世界, 症状是"改了生成器但世界没变")
for pair in "worlds/work_area.sdf:$WORLD" "config/bridge.yml:$BRIDGE"; do
    rel="${pair%%:*}"; dst="${pair#*:}"
    src="$VLA_HOME/vla_ws/src/vla_follow/$rel"
    # install 是 --symlink-install 时 dst 本身就是指向 src 的符号链接, 不用拷
    if [ ! -L "$dst" ] && [ -f "$src" ] && [ "$src" -nt "$dst" ]; then
        cp "$src" "$dst" && echo ">>> 同步 $rel -> install"
    fi
done

GZARGS="-r -v 3 --render-engine ogre"
[ "$WITH_GUI" = "0" ] && GZARGS="$GZARGS -s"

setsid nohup gz sim $GZARGS "$WORLD" \
    > "$VLA_HOME/vla_ws/logs/gzsim.log" 2>&1 < /dev/null &
for i in $(seq 1 45); do
    if timeout 3 gz topic -l 2>/dev/null | grep -q dynamic_pose; then
        echo ">>> gz topics 出现 (${i}s)"
        break
    fi
    sleep 1
done
sleep 3

setsid nohup ros2 run ros_gz_bridge parameter_bridge --ros-args \
    -p config_file:="$BRIDGE" \
    > "$VLA_HOME/vla_ws/logs/bridge.log" 2>&1 < /dev/null &
sleep 4

echo ">>> 关节指令话题: $(gz topic -l 2>/dev/null | grep -c '/drink/cmd/')"
echo ">>> gz 错误行: $(grep -c '\[Err\]' "$VLA_HOME/vla_ws/logs/gzsim.log")"
grep '\[Err\]' "$VLA_HOME/vla_ws/logs/gzsim.log" | head -5
echo ">>> 桥接: $(grep -c . "$VLA_HOME/vla_ws/logs/bridge.log") 行日志"
