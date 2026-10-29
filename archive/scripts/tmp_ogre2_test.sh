#!/bin/bash
# 测试 ogre2 + 软件 GL 能否稳定渲染 3D 视图
set +u
pkill -9 -f '[r]os2 launch vla_follow' 2>/dev/null
pkill -9 -f '[v]la_follow/lib' 2>/dev/null
pkill -9 -f '[p]arameter_bridge' 2>/dev/null
pkill -9 -f '[g]z sim' 2>/dev/null
pkill -9 -f '[g]z_viz' 2>/dev/null
sleep 5
echo "残留: $(pgrep -f '[g]z sim' | wc -l)"

sed -i 's#<engine>ogre</engine>#<engine>ogre2</engine>#' ~/.gz/sim/8/gui.config
grep engine ~/.gz/sim/8/gui.config | head -2

cd /home/earl/workspace/VLA || exit 1
LIBGL_ALWAYS_SOFTWARE=1 DISPLAY=:0 setsid nohup gz sim -r --render-engine ogre2 \
  /home/earl/workspace/VLA/vla_ws/install/vla_follow/share/vla_follow/worlds/work_area.sdf \
  > /tmp/gz_ogre2.log 2>&1 < /dev/null &
sleep 35
DISPLAY=:0 ffmpeg -y -loglevel error -f x11grab -i :0 -frames:v 1 /tmp/cam_ogre2.png
ls -l /tmp/cam_ogre2.png
echo DONE
