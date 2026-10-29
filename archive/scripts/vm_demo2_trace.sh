#!/usr/bin/env bash
# vm_demo2_trace.sh —— 用 demo2 的真实推理路径(LocalPolicy)逐步记录闭环行为
cd /home/earl/vla/code/test
python3 - <<'EOF'
import sys, time
sys.path.insert(0, ".")
sys.path.insert(0, "../mini_vla")
import numpy as np
from gz_env import GzArenaEnv
from robot_loop import LocalPolicy
import env_synth as E

env = GzArenaEnv()
policy = LocalPolicy("runs/gz_model/ckpt.pt")

for seed in (5000, 5001, 5002):
    env.reset("reach", seed)
    st = env.state_px()
    tgt = st["red"].copy()
    print(f"--- seed {seed} 初始距离={np.linalg.norm(st['agent']-tgt):.1f}")
    for t in range(30):
        obs = env.get_obs()
        chunk = policy.act(obs)                     # 未归一化 (H,2) px/s
        a = np.clip(chunk[0], -E.MAX_SPEED, E.MAX_SPEED)
        ok = env.step(a)
        st = env.state_px()
        if t % 3 == 0 or ok:
            print(f"  t={t:02d} dist={np.linalg.norm(st['agent']-tgt):5.1f} "
                  f"|a|={np.linalg.norm(a):5.1f} a={a.round(1).tolist()}", flush=True)
        if ok:
            print(f"  SUCCESS at t={t}")
            break
    env.stop()
EOF
