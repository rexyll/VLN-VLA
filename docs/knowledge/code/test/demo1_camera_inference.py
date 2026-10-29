# -*- coding: utf-8 -*-
"""demo1 —— 虚拟相机 + 推理服务联通验证

验证链路: Gazebo 虚拟相机帧/状态 → VLA 推理 → 动作块
检查项:  ① 图像感知与桥接真值一致（色块质心 vs 真值位置, <2.5px）
         ② 推理输出动作块与专家参考方向一致（余弦 > 0.8）
         ③ 保存一帧快照供人工检查
运行: python3 demo1_camera_inference.py --ckpt runs/gz_model/ckpt.pt
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


def blob(img: np.ndarray, cond) -> np.ndarray | None:
    ys, xs = np.where(cond)
    if len(xs) < 3:
        return None
    return np.array([xs.mean(), ys.mean()])  # (x_px, y_px)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ckpt", default="runs/gz_model/ckpt.pt")
    ap.add_argument("--task", default="reach", choices=["reach", "push"])
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--save", default="results/demo1_frame.png")
    args = ap.parse_args()

    print("=== DEMO 1: 虚拟相机 + 推理服务联通验证 ===")
    env = GzArenaEnv()
    env.reset(args.task, args.seed)
    obs = env.get_obs()
    img = obs["image"]

    # ① 相机感知 vs 桥接真值（阈值针对自动增益后的图像，颜色被压暗约 3 倍）
    st = env.state_px()
    blue = blob(img, (img[:, :, 2].astype(int) - img[:, :, 0] > 18)
                & (img[:, :, 2].astype(int) - img[:, :, 1] > 18) & (img[:, :, 2] > 35))
    tgt = st["red"] if args.task == "reach" else st["yellow"]
    tcol = (img[:, :, 0].astype(int) - img[:, :, 1] > 18) & (img[:, :, 0].astype(int) - img[:, :, 2] > 18) \
        & (img[:, :, 0] > 35) \
        if args.task == "reach" else \
        (img[:, :, 1].astype(int) - img[:, :, 2] > 20) & (img[:, :, 0] > 35) & (img[:, :, 1] > 30)
    cross = blob(img, tcol)
    e_agent = np.linalg.norm(blue - st["agent"]) if blue is not None else 99
    e_tgt = np.linalg.norm(cross - tgt) if cross is not None else 99
    print(f"[1] 相机感知: agent 质心误差={e_agent:.2f}px  目标十字质心误差={e_tgt:.2f}px")

    # ② 保存快照
    p = pathlib.Path(args.save)
    p.parent.mkdir(parents=True, exist_ok=True)
    from PIL import Image
    Image.fromarray(img).save(p)
    print(f"[2] 快照已保存: {p}")

    # ③ 推理
    from robot_loop import LocalPolicy
    policy = LocalPolicy(args.ckpt)
    chunk = policy.act(obs)
    expert = E.expert_action(obs["state"], obs["box"], env.target,
                             args.task, np.random.default_rng(0))
    cos = float(np.dot(chunk[0], expert) /
                (np.linalg.norm(chunk[0]) * np.linalg.norm(expert) + 1e-9))
    print(f"[3] 推理: chunk[0]={np.round(chunk[0], 1).tolist()} px/s  "
          f"专家参考={np.round(expert, 1).tolist()}  方向余弦={cos:.3f}")

    ok = e_agent < 2.5 and e_tgt < 2.5 and cos > 0.8
    print(f"RESULT: {'PASS' if ok else 'CHECK'}  "
          f"(阈值: 质心误差<2.5px, 余弦>0.8)")


if __name__ == "__main__":
    main()
