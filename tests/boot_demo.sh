#!/bin/bash
# tests/boot_demo.sh [seed] —— 干净重启 demo(不跑评测), 打印就绪状态
#
# 位置: tests/   依赖: vla_demo/start_demo.sh
set +u
export VLA_HOME="${VLA_HOME:-/home/earl/workspace/VLA}"
cd "$VLA_HOME" || exit 1
source /opt/ros/jazzy/setup.bash
source "$VLA_HOME/vla_ws/install/setup.bash"
SEED=${1:-7}
OUTDIR="$VLA_HOME/tests/out"
mkdir -p "$OUTDIR"

pkill -9 -f 'ros2 launch vla_follow' 2>/dev/null
pkill -9 -f 'vla_follow/lib' 2>/dev/null
pkill -9 -f parameter_bridge 2>/dev/null
pkill -9 -f '/opt/ros/jazzy/lib/nav2_' 2>/dev/null
pkill -9 -f 'gz sim' 2>/dev/null
sleep 3
for i in $(seq 1 40); do
    [ "$(pgrep -f "[g]z sim" | wc -l)" -eq 0 ] && break
    sleep 1
done
if [ "$(pgrep -f "[g]z sim" | wc -l)" -ne 0 ]; then
    echo ">>> 清场失败"; exit 3
fi
env DISPLAY=:0 PW_SEED=$SEED nohup bash "$VLA_HOME/vla_demo/start_demo.sh" \
    > "$OUTDIR/demo_boot.log" 2>&1 < /dev/null &
for i in $(seq 1 30); do
    sleep 3
    if ros2 node list 2>/dev/null | grep -q /person_vision; then
        echo ">>> demo 就绪"; break
    fi
done
sleep 4
echo ">>> 节点数 $(ros2 node list 2>/dev/null | grep -c .)"
tail -3 "$OUTDIR/demo_boot.log"
