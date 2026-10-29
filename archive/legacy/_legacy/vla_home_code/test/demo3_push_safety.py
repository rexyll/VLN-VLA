# -*- coding: utf-8 -*-
"""demo3 —— push 任务闭环 + 安全层/看门狗验证

A) push 任务闭环 5 集（更难的任务 + 虚拟接触传感器读数）
B) 推理中断 → 看门狗受控停止（模拟推理服务崩溃）
C) 越限动作 → 安全层裁剪（下发 999 px/s）
运行: python3 demo3_push_safety.py --ckpt runs/gz_model/ckpt.pt
"""
from __future__ import annotations

import argparse
import pathlib
import sys
import time

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "mini_vla"))
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import env_synth as E  # noqa: E402
from gz_env import GzArenaEnv  # noqa: E402


class FlakyPolicy:
    """在第 fail_at 次调用后抛异常，模拟推理服务崩溃。"""

    def __init__(self, inner, fail_at: int):
        self.inner, self.fail_at, self.n = inner, fail_at, 0

    def act(self, obs):
        self.n += 1
        if self.n > self.fail_at:
            raise RuntimeError("simulated inference crash")
        return self.inner.act(obs)


def closed_loop(env, policy, task, episodes, max_steps=60, seed0=9000):
    results, max_contact = [], 0
    for ep in range(episodes):
        env.reset(task, seed0 + ep)
        ok, steps = False, max_steps
        for t in range(max_steps):
            obs = env.get_obs()
            max_contact = max(max_contact, obs["contact"])
            chunk = policy.act(obs)
            if env.step(np.clip(chunk[0], -E.MAX_SPEED, E.MAX_SPEED)):
                ok, steps = True, t + 1
                break
        results.append((ok, steps))
        print(f"  ep{ep:02d}: {'SUCCESS' if ok else 'FAIL'}  steps={steps}", flush=True)
    return results, max_contact


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ckpt", default="runs/gz_model/ckpt.pt")
    ap.add_argument("--episodes", type=int, default=5)
    args = ap.parse_args()

    print("=== DEMO 3: push 任务 + 安全层/看门狗验证 ===")
    env = GzArenaEnv()
    from robot_loop import LocalPolicy, SafetyLayer
    policy = LocalPolicy(args.ckpt)
    safety = SafetyLayer()
    all_ok = True

    # ---- A) push 闭环 ----
    print("[A] push 任务闭环:")
    res, max_contact = closed_loop(env, policy, "push", args.episodes)
    sr = float(np.mean([r[0] for r in res]))
    print(f"[A] push 成功率={sr*100:.1f}%  接触传感器峰值={max_contact}")

    # ---- B) 看门狗：推理中断 → 受控停止 ----
    print("[B] 看门狗测试（第 4 步起推理崩溃）:")
    env.reset("reach", 777)
    flaky = FlakyPolicy(policy, fail_at=3)
    stopped_ok = False
    for t in range(30):
        obs = env.get_obs()
        try:
            chunk = flaky.act(obs)
        except RuntimeError as exc:
            p_before = env.state_px()["agent"].copy()
            env.stop()                                   # 看门狗动作：受控停止
            time.sleep(0.6)
            p_after = env.state_px()["agent"]
            drift = float(np.linalg.norm(p_after - p_before))
            stopped_ok = drift < 0.8
            print(f"    推理中断于 t={t}: 停止前={p_before.round(1).tolist()} "
                  f"停止后={p_after.round(1).tolist()} 漂移={drift:.2f}px -> "
                  f"{'受控停止生效' if stopped_ok else '未停住!'}")
            break
        env.step(np.clip(chunk[0], -E.MAX_SPEED, E.MAX_SPEED))
    all_ok &= stopped_ok

    # ---- C) 安全层越限裁剪 ----
    print("[C] 安全层测试（下发 [999, -999] px/s）:")
    vel, reason = safety.check([999, -999], [32.0, 32.0])
    clipped = vel is not None and np.abs(vel).max() <= E.MAX_SPEED
    print(f"    裁剪后={np.round(vel, 1).tolist() if vel is not None else None} "
          f"拒绝原因={reason} -> {'限速生效' if clipped else '异常!'}")
    all_ok &= clipped
    nan_vel, nan_reason = safety.check([float("nan"), 1.0], [32.0, 32.0])
    print(f"    NaN 动作 -> 拒绝原因={nan_reason} -> "
          f"{'拦截生效' if nan_vel is None else '异常!'}")
    all_ok &= nan_vel is None

    print(f"RESULT: {'PASS' if all_ok and sr >= 0.6 else 'CHECK'}")


if __name__ == "__main__":
    main()
