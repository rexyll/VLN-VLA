# -*- coding: utf-8 -*-
"""demo2 —— VLA 闭环控制验证（reach 任务，N 集滚动时域 rollout）

链路: Gazebo 相机观测 → VLA 策略 → 速度指令 → 仿真物理 → 成功判定
运行: python3 demo2_closed_loop.py --ckpt runs/gz_model/ckpt.pt --episodes 10
"""
from __future__ import annotations

import argparse
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "mini_vla"))
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import env_synth as E  # noqa: E402
from gz_env import GzArenaEnv  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ckpt", default="runs/gz_model/ckpt.pt")
    ap.add_argument("--task", default="reach", choices=["reach", "push"])
    ap.add_argument("--episodes", type=int, default=10)
    ap.add_argument("--max-steps", type=int, default=60)
    ap.add_argument("--seed0", type=int, default=5000)
    args = ap.parse_args()

    print(f"=== DEMO 2: VLA 闭环控制验证 (task={args.task}) ===")
    env = GzArenaEnv()
    from robot_loop import LocalPolicy, SafetyLayer
    policy = LocalPolicy(args.ckpt)
    safety = SafetyLayer()

    results = []
    for ep in range(args.episodes):
        env.reset(args.task, args.seed0 + ep)
        ok, steps = False, args.max_steps
        for t in range(args.max_steps):
            obs = env.get_obs()
            chunk = policy.act(obs)
            vel, _ = safety.check(chunk[0], obs["state"])   # 安全层：限速/合法性
            if vel is None:
                env.stop()
                continue
            if env.step(vel):
                ok, steps = True, t + 1
                break
        results.append((ok, steps))
        print(f"ep{ep:02d}: {'SUCCESS' if ok else 'FAIL'}  steps={steps}", flush=True)

    sr = float(np.mean([r[0] for r in results]))
    from evaluate import wilson_ci
    lo, hi = wilson_ci(sr, args.episodes)
    avg = float(np.mean([r[1] for r in results]))
    print(f"成功率 = {sr*100:.1f}%  Wilson95=[{lo*100:.1f}, {hi*100:.1f}]  "
          f"平均步数={avg:.1f}")
    print(f"RESULT: {'PASS' if sr >= 0.7 else 'FAIL'}  (门禁: SR>=70%)")


if __name__ == "__main__":
    main()
