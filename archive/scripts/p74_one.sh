#!/bin/bash
# p74_one.sh <tag> <dur> —— 单轮确定性录制
# 关键: pgrep -f 'gz sim' 会匹配到自己的命令行, 门禁永远不过; 用 pgrep -x gz。
set +u
cd /home/earl/workspace/VLA || exit 1
TAG=${1:-a}
DUR=${2:-100}

echo ">>> [$TAG] 清场"
pkill -9 -f 'ros2 launch vla_follow' 2>/dev/null
pkill -9 -f 'vla_follow/lib' 2>/dev/null
pkill -9 -f parameter_bridge 2>/dev/null
pkill -9 -f '/opt/ros/jazzy/lib/nav2_' 2>/dev/null
pkill -9 -f 'gz sim' 2>/dev/null
pkill -9 -f tmp_p74_det 2>/dev/null
pkill -9 -f tmp_set_follow 2>/dev/null
sleep 4

for i in $(seq 1 40); do
    g=$(pgrep -x gz | wc -l)
    b=$(pgrep -x parameter_bridge | wc -l)
    if [ "$g" -eq 0 ] && [ "$b" -eq 0 ]; then
        echo ">>> [$TAG] 清场完成 (${i}s)"
        break
    fi
    sleep 1
done
echo ">>> [$TAG] 残留 gz=$(pgrep -x gz | wc -l) bridge=$(pgrep -x parameter_bridge | wc -l)"

rm -f "det_$TAG.json" "det_$TAG.log"
echo ">>> [$TAG] 起录制器"
setsid nohup bash p74_rec.sh "$DUR" "det_$TAG.json" > "det_$TAG.log" 2>&1 < /dev/null &

sleep 3
echo ">>> [$TAG] 起 demo"
DISPLAY=:0 PW_SEED=7 nohup bash vla_demo/start_demo.sh > "demo_$TAG.log" 2>&1 < /dev/null &

sleep 32
setsid nohup python3 tmp_set_follow.py 1.0 $((DUR + 40)) > /dev/null 2>&1 < /dev/null &

for i in $(seq 1 70); do
    sleep 5
    if grep -q 'wrote' "det_$TAG.log" 2>/dev/null; then
        echo ">>> [$TAG] 录制完成 ($(($i * 5))s 轮询)"
        break
    fi
done
pkill -f tmp_set_follow 2>/dev/null
echo ">>> [$TAG] det_$TAG.log:"
cat "det_$TAG.log"
