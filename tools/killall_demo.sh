#!/bin/bash
# killall_demo.sh —— 清场: 杀掉所有 demo 相关进程(含孤儿)
#
# 为什么单独成脚本而不是 ssh 一行命令:
#   `pkill -f 'ros2 launch vla_follow'` 的模式串会出现在**自己这条命令行**里,
#   pkill -f 匹配整条 cmdline → 把自己的 shell 一起 -9 掉, 远程命令直接 RC=-1,
#   后面的清理一条都不执行(排查了两轮才发现)。放进脚本文件后, 进程 cmdline
#   只有脚本路径, 不会自匹配。
#
# 2026-10-26 又一个坑(代价: 排查 3 小时):
#   `gz sim` 是 Ruby 启动器, **进程 comm 是 `ruby` 而不是 `gz`**。
#   原来写的 `pkill -9 -x gz` 和 `pkill -9 -f 'gz-sim'`(连字符) 两个都不匹配,
#   每次 start_scene 都静默留下一个完整世界。19 个仿真同时广播
#   /world/follow_world/dynamic_pose/info, 订阅方把 19 份位姿交错着读,
#   于是"水壶被掀飞到 -6m / 水杯飞到 +2.6m"—— 数据是真实的, 但属于不同的
#   世界。必须按 cmdline 里的 "gz sim" 来杀。
#
# 用法: bash tools/killall_demo.sh [--keep-gz]

KEEP_GZ=0
[ "$1" = "--keep-gz" ] && KEEP_GZ=1

pkill -9 -f 'ros2 launch vla_follow' 2>/dev/null
pkill -9 -f 'vla_follow/lib/vla_follow' 2>/dev/null
pkill -9 -f 'parameter_bridge' 2>/dev/null
pkill -9 -f '/opt/ros/jazzy/lib/nav2_' 2>/dev/null
pkill -9 -f 'ros2 topic hz' 2>/dev/null
pkill -9 -f 'tests/diag/' 2>/dev/null
pkill -9 -f 'tools/pose_demo.py' 2>/dev/null
sleep 2

if [ "$KEEP_GZ" = "0" ]; then
    pkill -9 -f 'gz sim' 2>/dev/null          # 真正的进程名(ruby + 自定义标题)
    pkill -9 -f 'gz-sim' 2>/dev/null
    pkill -9 -f 'gz-sim-server' 2>/dev/null
    pkill -9 -f 'ign gazebo' 2>/dev/null
    pkill -9 -x gz 2>/dev/null
    sleep 2
fi

# 残留统计: 直接数 cmdline 里带 gz sim 的进程, 不看 comm
left=$(ps -eo args --no-headers \
    | grep -E 'gz sim|vla_follow/lib|parameter_bridge|nav2_|tests/diag' \
    | grep -v grep | grep -v killall_demo | wc -l)
echo ">>> 清理完成, 残留进程数: ${left}"
if [ "$left" != "0" ]; then
    ps -eo pid,args --no-headers \
        | grep -E 'gz sim|vla_follow/lib|parameter_bridge|nav2_' \
        | grep -v grep | grep -v killall_demo | cut -c1-110
fi
