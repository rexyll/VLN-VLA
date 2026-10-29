#!/bin/bash
# tmp_nav_manual.sh —— 手动驱动剩余 lifecycle 节点(管理器卡死时的运维手段)
source /opt/ros/jazzy/setup.bash
for n in controller_server behavior_server bt_navigator; do
  st=$(timeout 5 ros2 lifecycle get /$n 2>/dev/null)
  echo "== $n: $st"
  case "$st" in
    *unconfigured*)
      timeout 30 ros2 service call /$n/configure lifecycle_msgs/srv/ChangeState \
        "{transition: {id: 1}}" >/dev/null 2>&1
      ;;
  esac
  timeout 30 ros2 service call /$n/activate lifecycle_msgs/srv/ChangeState \
    "{transition: {id: 3}}" >/dev/null 2>&1
  echo "   → $(timeout 5 ros2 lifecycle get /$n 2>/dev/null)"
done
echo MANUAL-DONE
