#!/bin/bash
# P76c: 重启后连抓 3 帧, 验证相机固定在 13m 全景(不再跟随机器人)
set +u
pkill -9 -f '[r]os2 launch vla_follow' 2>/dev/null
pkill -9 -f '[v]la_follow/lib' 2>/dev/null
pkill -9 -f '[p]arameter_bridge' 2>/dev/null
pkill -9 -f '[g]z sim' 2>/dev/null
pkill -9 -f '[g]z_viz' 2>/dev/null
sleep 6

cd /home/earl/workspace/VLA || exit 1
python3 vla_demo/gz_gui_fix.py
rm -f vla_ws/logs/demo.log
PW_REPLAY=/home/earl/workspace/VLA/walk_rec.jsonl DISPLAY=:0 \
  setsid nohup bash vla_demo/start_demo.sh > demo_final.log 2>&1 < /dev/null &

for i in $(seq 1 60); do
    sleep 2
    grep -q 'PERSON 行走' vla_ws/logs/demo.log 2>/dev/null && break
done
echo "人开始行走: ${i} 次轮询($((i*2))s)"

for t in 20 40 60; do
    sleep 20
    DISPLAY=:0 ffmpeg -y -loglevel error -f x11grab -i :0 -frames:v 1 /tmp/fin_$t.png
    echo "t=$t  $(stat -c%s /tmp/fin_$t.png) bytes"
done

echo "--- 进程: gz=$(pgrep -f '[g]z sim' | wc -l) viz=$(pgrep -f '[g]z_viz' | wc -l) flw=$(pgrep -f '[f]ollower' | wc -l)"
echo "--- VIZ 日志:"; grep -a 'VIZ' vla_ws/logs/demo.log | tail -4
echo DONE
