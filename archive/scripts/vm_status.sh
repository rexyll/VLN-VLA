#!/usr/bin/env bash
# vm_status.sh —— 全面检查 VM 侧任务状态
echo "=== 相关进程 ==="
pgrep -af "gz sim|collect_gazebo|train_gz|python3" | grep -v grep | head -8 || echo "无相关进程"
echo "=== gzserver 进程 ==="
ps -o pid,etime,%cpu,%mem,cmd -C gz 2>/dev/null | head -4
ps -o pid,etime,%cpu,%mem,cmd -p $(pgrep -f "gz sim" | head -1) 2>/dev/null | tail -1
echo "=== 产物文件 ==="
ls -la /home/earl/vla/code/test/gz_data.npz 2>/dev/null || echo "gz_data.npz 不存在"
ls -la /home/earl/vla/code/test/runs/gz_model/ckpt.pt 2>/dev/null || echo "gz_model ckpt 不存在"
ls -la /home/earl/vla/code/test/results/ 2>/dev/null | head -5
echo "=== gzserver 日志尾部 ==="
tail -6 /home/earl/vla/code/test/gzserver.log 2>/dev/null
echo "=== 桥接探测（独立短连接） ==="
python3 - <<'EOF'
import socket
try:
    s = socket.create_connection(("127.0.0.1", 9101), timeout=4)
    s.sendall(b"PING\n")
    print("BRIDGE:", s.recv(100).decode().strip())
except Exception as e:
    print("BRIDGE-FAIL:", e)
EOF
echo "=== 内存/负载 ==="
free -h | sed -n 2p
uptime
