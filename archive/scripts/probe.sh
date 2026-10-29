#!/usr/bin/env bash
# probe.sh —— 探测 vla_bridge 端口与相机 topic 消息类型
python3 - <<'EOF'
import socket
try:
    s = socket.create_connection(("127.0.0.1", 9101), timeout=5)
    s.sendall(b"PING\n")
    print("BRIDGE:", s.recv(100).decode().strip())
except Exception as e:
    print("BRIDGE-FAIL:", e)
EOF
source /usr/share/gz/gz1/setup.sh 2>/dev/null
echo topic_vla_cam:
gz topic -i /vla_cam 2>/dev/null | head -3
echo topic_side:
gz topic -i /gz_side_cam 2>/dev/null | head -3
