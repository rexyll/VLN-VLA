#!/bin/bash
# P76 收尾: 干净重启 demo, 恢复默认 GUI 布局 (front 相机小窗 + 3D 视图)
set +u
pkill -9 -f '[r]os2 launch vla_follow' 2>/dev/null
pkill -9 -f '[v]la_follow/lib' 2>/dev/null
pkill -9 -f '[p]arameter_bridge' 2>/dev/null
pkill -9 -f '[g]z sim' 2>/dev/null
pkill -9 -f '[g]z_viz' 2>/dev/null
sleep 6
echo "残留 gz=$(pgrep -f '[g]z sim' | wc -l) follower=$(pgrep -f '[f]ollower' | wc -l)"

# 先把 gui.config 里的 ImageDisplay 强制写回 /camera (上次跑的是 /overhead/image)
cd /home/earl/workspace/VLA || exit 1
python3 vla_demo/gz_gui_fix.py
echo "--- gui.config 关键项:"
grep -o '<engine>[^<]*</engine>' ~/.gz/sim/8/gui.config | head -1
grep -o '<camera_pose>[^<]*</camera_pose>' ~/.gz/sim/8/gui.config | head -1
grep -c '<topic>/camera</topic>' ~/.gz/sim/8/gui.config

PW_REPLAY=/home/earl/workspace/VLA/walk_rec.jsonl DISPLAY=:0 \
  setsid nohup bash vla_demo/start_demo.sh > demo_final.log 2>&1 < /dev/null &

for i in $(seq 1 40); do
    sleep 2
    grep -q 'PERSON 行走' vla_ws/logs/demo.log 2>/dev/null && break
done
echo "--- 启动到人行走用了约 $((i*2))s"

sleep 20
DISPLAY=:0 ffmpeg -y -loglevel error -f x11grab -i :0 -frames:v 1 /tmp/fin_t25.png
echo "--- t25 截图: $(ls -l /tmp/fin_t25.png | awk '{print $5}') bytes"

sleep 25
DISPLAY=:0 ffmpeg -y -loglevel error -f x11grab -i :0 -frames:v 1 /tmp/fin_t50.png
echo "--- t50 截图: $(ls -l /tmp/fin_t50.png | awk '{print $5}') bytes"

echo "--- 进程: gz=$(pgrep -f '[g]z sim' | wc -l) viz=$(pgrep -f '[g]z_viz' | wc -l) flw=$(pgrep -f '[f]ollower' | wc -l)"
echo "--- diag 抽样:"
timeout 8 ros2 topic echo --once /follower/diag 2>/dev/null | head -4
echo DONE
