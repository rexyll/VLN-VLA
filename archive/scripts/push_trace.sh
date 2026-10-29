#!/usr/bin/env bash
# vm_push_trace.sh —— push 专家在仿真中的逐步轨迹诊断
cd /home/earl/vla/code/test
python3 - <<'EOF'
import sys, time
sys.path.insert(0, ".")
from gz_env import GzArenaEnv
import env_synth as E
import numpy as np

env = GzArenaEnv()
succeeded = 0
for ep in range(6):
    seed = 100000 + ep
    rng = np.random.default_rng(seed + 777)
    env.reset("push", seed)
    tgt, ag, bx = env.target.copy(), None, None
    st = env.state_px()
    ag, bx = st["agent"].copy(), st["box"].copy()
    trace = []
    ok = False
    for t in range(70):
        a = E.expert_action(ag, bx, tgt, "push", rng)
        env.vel(float(a[0]), float(a[1]))
        time.sleep(E.DT)
        st = env.state_px()
        ag, bx = st["agent"].copy(), st["box"].copy()
        d_box_tgt = float(np.linalg.norm(bx - tgt))
        d_ag_box = float(np.linalg.norm(ag - bx))
        trace.append((t, d_ag_box, d_box_tgt, int(st["contact"])))
        if d_box_tgt < 4.0:
            ok = True
            break
    succeeded += ok
    print(f"ep{ep}: {'OK' if ok else 'FAIL'} 终态 box-target={d_box_tgt:.1f} "
          f"agent-box={d_ag_box:.1f}")
    if not ok and ep == 0:
        for row in trace[::6]:
            print(f"   t={row[0]:2d} ag-box={row[1]:5.1f} box-tgt={row[2]:5.1f} c={row[3]}")
print(f"成功 {succeeded}/6")
EOF
