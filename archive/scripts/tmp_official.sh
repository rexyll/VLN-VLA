#!/bin/bash
# tmp_official.sh —— 在官方 diff_drive 示例世界上做差速响应对照实验
export DISPLAY=:0
pkill -f 'ros2 launch vla_follow' 2>/dev/null
sleep 1
pkill -f 'vla_follow/lib/vla_follow' 2>/dev/null
pkill -f 'gz sim' 2>/dev/null
sleep 3
pkill -9 -f 'gz sim' 2>/dev/null
sleep 1
setsid nohup gz sim -r -v 1 --render-engine ogre \
  /usr/share/gz/gz-sim8/worlds/diff_drive_skid.sdf \
  > /home/earl/tda/official.log 2>&1 < /dev/null &
sleep 16
echo '--- 官方示例: w=+1.0 ---'
python3 -u /home/earl/tda/tmp_official_test.py 0 1.0 3 2>&1 | tail -3
echo '--- 官方示例: v=+0.5 ---'
python3 -u /home/earl/tda/tmp_official_test.py 0.5 0 3 2>&1 | tail -2
echo DONE
