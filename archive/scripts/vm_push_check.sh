#!/usr/bin/env bash
# vm_push_check.sh —— 验证运动学推箱机制（修复后快速功能测试）
cd /home/earl/vla/code/test
python3 - <<'EOF'
import sys, time
sys.path.insert(0, ".")
from gz_env import GzArenaEnv

env = GzArenaEnv()
# 场景: 智能体(22,32) 箱子(32,32) 黄十字目标(48,32) 红十字干扰(56,8)
env._cmd("RESET 0.22 0.32 0.32 0.32 0.56 0.08 0.48 0.32")
time.sleep(0.5)
st = env.state_px()
print("initial:", "agent=", st["agent"].round(1).tolist(),
      "box=", st["box"].round(1).tolist(), flush=True)
env.vel(35, 0)
for i in range(8):
    time.sleep(0.2)
    st = env.state_px()
    print(f"t={(i+1)*0.2:.1f}s agent={st['agent'].round(1).tolist()} "
          f"box={st['box'].round(1).tolist()} contact={st['contact']}", flush=True)
env.stop()
time.sleep(0.3)
st = env.state_px()
moved = float(st["box"][0]) - 32
print("PUSH-CHECK:", "PASS" if moved > 8 else "FAIL", f"(box moved {moved:.1f}px)")
EOF
