#!/bin/bash
# p74_rec.sh —— 独立录制入口，供 setsid 后台调用
set +u
export DISPLAY=:0
source /opt/ros/jazzy/setup.bash
source /home/earl/workspace/VLA/vla_ws/install/setup.bash
cd /home/earl/workspace/VLA || exit 1
exec python3 tmp_p74_det.py --dur "$1" --out "$2"
