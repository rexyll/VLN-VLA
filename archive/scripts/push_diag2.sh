#!/usr/bin/env bash
# vm_push_diag2.sh —— 用采集器同款代码路径复现 push 采集（与独立轨迹对照）
cd /home/earl/vla/code/test
python3 - <<'EOF'
import sys, pathlib
sys.path.insert(0, ".")
sys.path.insert(0, "../mini_vla")
import numpy as np
import env_synth as E
from gz_env import GzArenaEnv
from collect_gazebo_data import collect_episode

env = GzArenaEnv()
for seed in (100000, 100001, 100002, 101000, 101001, 101002):
    try:
        out = collect_episode(env, "push", seed)
        imgs, ags, bxs, acts, tgt, done_t, L = out
        if done_t == E.T:
            print(f"seed {seed}: FAIL  L={L}")
        else:
            print(f"seed {seed}: OK    L={L} done_t={done_t}")
    except Exception as exc:
        print(f"seed {seed}: EXC {type(exc).__name__}: {exc}")
EOF
