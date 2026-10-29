#!/bin/bash
# tmp_nuke.sh —— 彻底清场(不使用会被自匹配的 pkill 命令行)
for pat in 'ros2 launch' 'parameter_bridge' 'vla_follow/lib' 'gz sim' 'gz-sim' 'nav2' 'cmd_mux' 'gt_tf' 'lifecycle_manager'; do
  pkill -f "$pat" 2>/dev/null
done
sleep 3
for pat in 'ros2 launch' 'parameter_bridge' 'vla_follow/lib' 'gz sim' 'nav2' 'cmd_mux' 'gt_tf' 'lifecycle_manager'; do
  pkill -9 -f "$pat" 2>/dev/null
done
sleep 2
echo "=== 清场后残留 ==="
ps -ef | grep -E 'ros2|gz sim|nav2|vla_follow|nav:' | grep -v grep | wc -l
