#!/bin/bash
# p76_scn_run.sh —— 受控场景: 人绕立柱走并在柱旁站定
set +u
cd /home/earl/workspace/VLA || exit 1
OUT=scn_pillar

pkill -9 -f '[r]os2 launch vla_follow' 2>/dev/null
pkill -9 -f '[v]la_follow/lib' 2>/dev/null
pkill -9 -f '[p]arameter_bridge' 2>/dev/null
pkill -9 -f '[o]pt/ros/jazzy/lib/nav2_' 2>/dev/null
pkill -9 -f '[g]z sim' 2>/dev/null
pkill -9 -f '[t]mp_p76_diag' 2>/dev/null
pkill -9 -f '[t]mp_set_follow' 2>/dev/null
sleep 4
for i in $(seq 1 40); do
    [ "$(pgrep -f '[g]z sim' | wc -l)" -eq 0 ] && break
    sleep 1
done
[ "$(pgrep -f '[g]z sim' | wc -l)" -ne 0 ] && { echo ">>> 清场失败"; exit 3; }

rm -f diag_$OUT.jsonl shot_a.png shot_b.png
echo ">>> 起 demo (受控场景回放)"
DISPLAY=:0 PW_REPLAY=/tmp/scn_pillar.jsonl \
    nohup bash vla_demo/start_demo.sh > "demo_$OUT.log" 2>&1 < /dev/null &

for i in $(seq 1 30); do
    sleep 3
    grep -q 'PERSON(回放)' vla_ws/logs/demo.log 2>/dev/null && break
done
sleep 5
echo ">>> 起诊断录制"
setsid nohup bash -c "source /opt/ros/jazzy/setup.bash && source /home/earl/workspace/VLA/vla_ws/install/setup.bash && python3 tmp_p76_diag.py --out /home/earl/workspace/VLA/diag_$OUT.jsonl --dur 150" > "diag_$OUT.log" 2>&1 < /dev/null &
setsid nohup python3 tmp_set_follow.py 1.0 180 > /dev/null 2>&1 < /dev/null &

# 人在绕柱途中 vs 已站定后, 各抓一张
(sleep 22; DISPLAY=:0 ffmpeg -y -loglevel error -f x11grab -video_size 1280x800 -i :0.0 -frames:v 1 /home/earl/workspace/VLA/shot_a.png >/dev/null 2>&1 </dev/null) &
(sleep 75; DISPLAY=:0 ffmpeg -y -loglevel error -f x11grab -video_size 1280x800 -i :0.0 -frames:v 1 /home/earl/workspace/VLA/shot_b.png >/dev/null 2>&1 </dev/null) &

sleep 95
pkill -f '[t]mp_p76_diag' 2>/dev/null
pkill -f '[t]mp_set_follow' 2>/dev/null
echo ">>> diag: $(wc -l < diag_$OUT.jsonl 2>/dev/null) 行"
ls -la shot_a.png shot_b.png 2>/dev/null
