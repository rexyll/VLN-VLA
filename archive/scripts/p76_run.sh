#!/bin/bash
# p76_run.sh <out> <dur> —— 回放口径评测 + 10Hz 诊断 + Gazebo 截图
set +u
cd /home/earl/workspace/VLA || exit 1
OUT=${1:-t17_p76}
DUR=${2:-120}
WALK=walk_rec.jsonl

pkill -9 -f 'ros2 launch vla_follow' 2>/dev/null
pkill -9 -f 'vla_follow/lib' 2>/dev/null
pkill -9 -f parameter_bridge 2>/dev/null
pkill -9 -f '/opt/ros/jazzy/lib/nav2_' 2>/dev/null
pkill -9 -f 'gz sim' 2>/dev/null
pkill -9 -f tmp_p76_diag 2>/dev/null
pkill -9 -f tmp_set_follow 2>/dev/null
sleep 4
for i in $(seq 1 40); do
    [ "$(pgrep -f "[g]z sim" | wc -l)" -eq 0 ] && break
    sleep 1
done
[ "$(pgrep -f "[g]z sim" | wc -l)" -ne 0 ] && { echo ">>> 清场失败"; exit 3; }

rm -f diag_$OUT.jsonl shot_$OUT.png
echo ">>> 起 demo (PW_REPLAY)"
DISPLAY=:0 PW_REPLAY=/home/earl/workspace/VLA/$WALK \
    nohup bash vla_demo/start_demo.sh > "demo_$OUT.log" 2>&1 < /dev/null &

for i in $(seq 1 30); do
    sleep 3
    grep -q 'PERSON(回放)' vla_ws/logs/demo.log 2>/dev/null && break
done
sleep 6
echo ">>> VIZ 就绪检查:"
grep -m2 -E 'VIZ 已创建|VIZ 就绪|gz_viz.*Error|gz_viz.*error' vla_ws/logs/demo.log 2>/dev/null | tail -3

echo ">>> 起诊断录制"
setsid nohup bash -c "source /opt/ros/jazzy/setup.bash && source /home/earl/workspace/VLA/vla_ws/install/setup.bash && python3 tmp_p76_diag.py --out /home/earl/workspace/VLA/diag_$OUT.jsonl --dur $((DUR+30))" > "diag_$OUT.log" 2>&1 < /dev/null &

# 评测进行到一半时抓一张 Gazebo 截图
(sleep $((DUR/2 + 20)); DISPLAY=:0 ffmpeg -y -loglevel error -f x11grab \
    -video_size $(DISPLAY=:0 xdpyinfo | grep -oE 'dimensions: *[0-9]+x[0-9]+' | grep -oE '[0-9]+x[0-9]+') \
    -i :0.0 -frames:v 1 "shot_$OUT.png" > /dev/null 2>&1 < /dev/null) &

echo ">>> 评测 $DUR s"
bash tmp_run_eval.sh "$DUR" "$OUT" 2>&1 | tail -3
sleep 3
pkill -f tmp_p76_diag 2>/dev/null
echo ">>> 结果:"
echo "  diag: $(wc -l < diag_$OUT.jsonl 2>/dev/null) 行"
ls -la shot_$OUT.png 2>/dev/null
