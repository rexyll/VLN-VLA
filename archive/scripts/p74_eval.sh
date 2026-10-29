#!/bin/bash
# p74_eval.sh <dur> <out> —— 干净重启 + 标准口径评测
set +u
cd /home/earl/workspace/VLA || exit 1
DUR=${1:-120}
OUT=${2:-t16_p74}

pkill -9 -f 'ros2 launch vla_follow' 2>/dev/null
pkill -9 -f 'vla_follow/lib' 2>/dev/null
pkill -9 -f parameter_bridge 2>/dev/null
pkill -9 -f '/opt/ros/jazzy/lib/nav2_' 2>/dev/null
pkill -9 -f 'gz sim' 2>/dev/null
pkill -9 -f tmp_set_follow 2>/dev/null
pkill -9 -f tmp_p74_det 2>/dev/null
sleep 4
for i in $(seq 1 40); do
    [ "$(pgrep -x gz | wc -l)" -eq 0 ] && break
    sleep 1
done

echo ">>> 起 demo (PW_SEED=7)"
DISPLAY=:0 PW_SEED=7 nohup bash vla_demo/start_demo.sh > "demo_$OUT.log" 2>&1 < /dev/null &

for i in $(seq 1 30); do
    sleep 3
    grep -q 'PERSON 行走' vla_ws/logs/demo.log 2>/dev/null && break
done
sleep 8
echo ">>> 清空日志起点标记"
wc -l < vla_ws/logs/demo.log > "/tmp/$OUT.mark"
echo ">>> 评测 $DUR s"
bash tmp_run_eval.sh "$DUR" "$OUT" 2>&1 | tail -3
echo ">>> 完成: $OUT.json"
