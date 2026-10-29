# -*- coding: utf-8 -*-
"""collect_gazebo_data.py —— 用脚本专家在 Gazebo 仿真里采集域内演示数据

用法: python3 collect_gazebo_data.py --n 60 --out gz_data.npz
产出: 与 mini_vla 数据集同 schema 的 npz，另含 images(E,T,64,64,3 uint8)。
      成功后仅追加 STOP_TAIL 帧"停止"演示（与 env_synth.STOP_TAIL 一致）。
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


def collect_episode(env: GzArenaEnv, task: str, seed: int):
    """采集一条演示；返回 (imgs, agents, boxes, actions, target, done_t, L)。"""
    rng = np.random.default_rng(seed + 777)
    env.reset(task, seed)
    imgs, ags, bxs, acts = [], [], [], []
    done_t = E.T
    min_d = None       # push 任务：箱-目标距离超过历史最近值 6px → 推离/卡死，快速判负
    for t in range(E.T):
        st = env.state_px()
        if done_t == E.T:
            a = E.expert_action(st["agent"], st["box"], env.target, task, rng)
        else:
            a = np.zeros(2, np.float32)
        img = env.image()  # 域差校正后的帧（与部署观测一致）
        imgs.append(img)
        ags.append(st["agent"].copy())
        bxs.append(st["box"].copy())
        acts.append(a.astype(np.float32))
        if done_t == E.T:
            if env.step(a):
                done_t = t
            elif task == "push":
                d = float(np.linalg.norm(st["box"] - env.target))
                min_d = d if (min_d is None or d < min_d) else min_d
                if d > min_d + 6.0:
                    break                      # 推离/卡死：快速判负，换种子重试
        elif t >= done_t + E.STOP_TAIL:
            break
    return (np.stack(imgs), np.stack(ags), np.stack(bxs), np.stack(acts),
            env.target.copy(), done_t, len(imgs))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=40, help="每任务成功演示条数")
    ap.add_argument("--out", default="gz_data.npz")
    ap.add_argument("--seed0", type=int, default=1000)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=9101)
    ap.add_argument("--max-minutes", type=float, default=25.0,
                    help="墙钟预算，到点保存已采集部分")
    args = ap.parse_args()

    env = GzArenaEnv(args.host, args.port)
    eps = []
    deadline = time.time() + args.max_minutes * 60
    for task in E.TASKS:
        got, seed, tried = 0, args.seed0 + (0 if task == "reach" else 100000), 0
        while got < args.n:
            if time.time() > deadline:
                print(f"[budget] 时间预算用完，{task} 已采集 {got} 条", flush=True)
                break
            tried += 1
            try:
                out = collect_episode(env, task, seed)
            except Exception as exc:      # 网络/桥接异常：自愈重连
                print(f"[warn] episode 异常({exc})，重连...", flush=True)
                try:
                    env.sock.close()
                except Exception:
                    pass
                time.sleep(3)
                try:
                    env = GzArenaEnv(args.host, args.port)
                except Exception:
                    time.sleep(5)
                continue
            seed += 1
            if out[5] == E.T:             # done_t == T: 专家失败，丢弃重试
                continue
            eps.append((*out, task, seed - 1))
            got += 1
            if got % 5 == 0:
                print(f"[{task}] {got}/{args.n} (尝试 {tried})", flush=True)

    if not eps:
        sys.exit("未采集到任何成功演示")
    Tmax = max(e[6] for e in eps)
    n = len(eps)
    print(f"共 {n} 条成功演示, 最长 {Tmax} 帧, 总帧数 {sum(e[6] for e in eps)}", flush=True)

    def pad(arr, L):
        return np.pad(arr, ((0, Tmax - L),) + ((0, 0),) * (arr.ndim - 1))

    np.savez_compressed(
        args.out,
        images=np.stack([pad(e[0], e[6]) for e in eps]),
        agent=np.stack([pad(e[1], e[6]) for e in eps]),
        box=np.stack([pad(e[2], e[6]) for e in eps]),
        actions=np.stack([pad(e[3], e[6]) for e in eps]),
        tlen=np.array([e[6] for e in eps], np.int32),
        success_t=np.array([e[5] for e in eps], np.int32),
        target=np.stack([e[4] for e in eps]),
        task=np.array([e[7] for e in eps]),
        text=np.array([E.TEXTS[e[7]][e[8] % len(E.TEXTS[e[7]])] for e in eps]),
        seed=np.array([e[8] for e in eps], np.int64),
        bg=np.zeros(n, np.int32),
    )
    print(f"saved -> {args.out}")


if __name__ == "__main__":
    main()
