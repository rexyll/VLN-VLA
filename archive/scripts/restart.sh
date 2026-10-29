#!/usr/bin/env bash
# vm_restart.sh —— 重建插件 + 干净重启仿真 + 探测桥接
set -e
bash /home/earl/vla/code/test/gazebo/build.sh
pkill -f "collect_gazebo_data" 2>/dev/null || true
pkill -f "gz sim" 2>/dev/null || true
pkill -f "xvfb-run" 2>/dev/null || true
for i in $(seq 1 20); do
  if ss -ltn 2>/dev/null | grep -q ":9101 "; then sleep 1; else break; fi
done
sleep 2
cd /home/earl/vla/code/test/gazebo
GZ_SIM_SYSTEM_PLUGIN_PATH=/home/earl/vla/code/test/gazebo LIBGL_ALWAYS_SOFTWARE=1 \
  nohup xvfb-run -a gz sim -s -r --verbose 2 vla_arena.world > ../gzserver.log 2>&1 &
echo "[restart] gzserver starting..."
sleep 30
echo "--- bridge/plugin 相关日志 ---"
grep -iaE "vla_bridge|failed|error|plugin" ../gzserver.log | head -25 || true
echo "--- 探测 ---"
bash /home/earl/vla/probe.sh
