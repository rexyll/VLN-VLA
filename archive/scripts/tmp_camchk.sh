#!/bin/bash
# 干净重启 demo 并立即截图，用于判断 GUI 相机是否在启动时就是俯视
set +u
cd /home/earl/workspace/VLA || exit 1
pkill -9 -f 'ros2 launch vla_follow' 2>/dev/null
pkill -9 -f 'vla_follow/lib' 2>/dev/null
pkill -9 -f parameter_bridge 2>/dev/null
pkill -9 -f '/opt/ros/jazzy/lib/nav2_' 2>/dev/null
pkill -9 -f 'gz sim' 2>/dev/null
pkill -9 -f tmp_set_follow 2>/dev/null
pkill -9 -f 'gz_viz' 2>/dev/null
sleep 4
for i in $(seq 1 30); do
    [ "$(pgrep -f "[g]z sim" | wc -l)" -eq 0 ] && break
    sleep 1
done
echo "残留: $(pgrep -f '[g]z sim' | wc -l)"

# 记录 GUI 重置前的 camera_pose
echo "--- 启动前 gui.config:"; grep camera_pose ~/.gz/sim/8/gui.config

env DISPLAY=:0 PW_REPLAY=/home/earl/workspace/VLA/walk_rec.jsonl \
    nohup bash vla_demo/start_demo.sh > demo_camchk.log 2>&1 < /dev/null &

for i in $(seq 1 40); do
    sleep 2
    grep -q 'PERSON(回放)' vla_ws/logs/demo.log 2>/dev/null && break
done
sleep 6
DISPLAY=:0 ffmpeg -y -loglevel error -f x11grab -i :0 -frames:v 1 /tmp/cam_t0.png
sleep 20
DISPLAY=:0 ffmpeg -y -loglevel error -f x11grab -i :0 -frames:v 1 /tmp/cam_t26.png
ls -l /tmp/cam_t0.png /tmp/cam_t26.png
echo "--- 启动后 gui.config:"; grep camera_pose ~/.gz/sim/8/gui.config
echo DONE
