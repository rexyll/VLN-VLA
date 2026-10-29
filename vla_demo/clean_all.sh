#!/bin/bash
# clean_all.sh —— 一键清场 (P71)
# 注意: 必须用 `pkill -f <完整路径前缀>` 而不是 `pkill -x <名字>` ——
#       Linux 的 comm 只有 15 字符, velocity_smoother/controller_server/
#       lifecycle_manager 会被截断成 velocity_smooth / controller_serv /
#       lifecycle_manag, 按全名精确匹配一个都杀不掉, 于是变成孤儿进程。
#       本脚本自己的命令行走的是 `bash clean_all.sh`, 不含下面任何模式,
#       所以 -f 不会自杀(在交互 shell 里直接敲才会, 故单独成脚本)。
echo ">>> 停 demo 进程"
pkill -f 'ros2 launch vla_follow' 2>/dev/null
pkill -f 'vla_follow/lib/vla_follow' 2>/dev/null
pkill -f parameter_bridge 2>/dev/null
pkill -f '/opt/ros/jazzy/lib/nav2_' 2>/dev/null
pkill -f 'gz sim' 2>/dev/null
pkill -f 'gz-sim' 2>/dev/null
sleep 2
pkill -9 -f '/opt/ros/jazzy/lib/nav2_' 2>/dev/null
pkill -9 -f 'gz sim' 2>/dev/null
pkill -9 -f 'gz-sim' 2>/dev/null
pkill -9 -f 'vla_follow/lib/vla_follow' 2>/dev/null
pkill -9 -f parameter_bridge 2>/dev/null
sleep 1
rm -f /tmp/vla_start_demo.lock
left=$(ps -eo args --no-headers | grep -E '/opt/ros/jazzy/lib/nav2_|gz sim|gz-sim|vla_follow/lib/vla_follow|parameter_bridge' | grep -v grep | wc -l)
echo ">>> 清场完成, 残留进程数: ${left}"
