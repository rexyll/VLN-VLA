#!/usr/bin/env bash
# vm_collect_check.sh —— 采集任务状态诊断
echo "--- process ---"
ps -o pid,etime,%cpu,%mem,cmd -p $(pgrep -f collect_gazebo | head -1) 2>/dev/null || echo NOT-RUNNING
echo "--- gz_data ---"
ls -la /home/earl/vla/code/test/gz_data.npz 2>/dev/null || echo NO-FILE
echo "--- bridge alive? ---"
python3 - <<'EOF'
import socket
try:
    s = socket.create_connection(("127.0.0.1", 9101), timeout=3)
    s.sendall(b"PING\n")
    print("BRIDGE:", s.recv(100).decode().strip())
    s.sendall(b"STATE\n")
    print("STATE:", s.recv(300).decode().strip()[:120])
except Exception as e:
    print("BRIDGE-FAIL:", e)
EOF
