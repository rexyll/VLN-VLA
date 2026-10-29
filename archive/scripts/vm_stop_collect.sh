#!/usr/bin/env bash
# vm_stop_collect.sh —— 安全停止采集进程
pkill -f "collect_gazebo_data" 2>/dev/null && echo collector-killed || echo no-collector
sleep 1
pgrep -f "collect_gazebo_data" >/dev/null && echo still-alive || echo stopped
