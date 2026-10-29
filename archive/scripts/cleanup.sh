#!/usr/bin/env bash
# vm_cleanup.sh —— 诊断负载来源 → 清理 → 确认恢复
echo "=== CPU 占用 TOP8（清理前） ==="
ps aux --sort=-%cpu | head -9
echo "=== 线程总数与每进程线程数 TOP5 ==="
echo "总线程数: $(ps -eLf | wc -l)"
ps -eLo pid,comm | grep -v PID | sort | uniq -c | sort -rn | head -5
echo "=== 清理：杀掉采集与仿真 ==="
pkill -f collect_gazebo && echo killed-collector || echo no-collector
pkill -f "gz sim" && echo killed-gz || echo no-gz
pkill -f xvfb-run && echo killed-xvfb || true
sleep 3
echo "=== 清理后负载 ==="
uptime
free -h | sed -n 2p
ps aux --sort=-%cpu | head -4
