# -*- coding: utf-8 -*-
"""
evaluate.py —— 合成环境 rollout 评估（docs/05 §5、docs/10 的可运行对应）

核心观点：离线 val loss 不是定版依据，必须用"预测→执行→判成功"的闭环成功率。

用法:
    python evaluate.py --ckpt runs/mini_vla/ckpt.pt --n 40

每任务默认 40 条（≥20 才有统计意义），报告成功率与 Wilson 95% 置信区间。
"""
from __future__ import annotations

import argparse
import json
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))  # 任意环境可导入同目录模块

import numpy as np
import torch

import env_synth as E
from dataset import encode_text, unnormalize
from model import MiniVLA, MiniVLAConfig


def load_model(ckpt_path: str, assets_dir: str):
    """加载 checkpoint 与部署四件套之二（norm_stats / vocab）。
    生产含义：模型、归一化统计、词表、提示模板必须同版本部署（docs/06 §7）。"""
    ckpt = torch.load(ckpt_path, map_location="cpu", weights_only=False)
    cfg = MiniVLAConfig(**ckpt["config"])
    model = MiniVLA(cfg)
    model.load_state_dict(ckpt["model"])
    model.eval()
    p = pathlib.Path(assets_dir)
    stats = json.loads((p / "norm_stats.json").read_text("utf-8"))
    vocab = json.loads((p / "vocab.json").read_text("utf-8"))
    meta = {"val_loss": ckpt.get("val_loss"), "epoch": ckpt.get("epoch")}
    return model, cfg, stats, vocab, meta


def sample_start(task: str, rng: np.random.Generator):
    """与训练数据同分布地采样初始条件（含约束重采样）。"""
    for _ in range(50):
        target = rng.uniform(10, E.S - 11, 2)
        agent = rng.uniform(6, E.S - 7, 2)
        box = rng.uniform(8, E.S - 9, 2)
        if (np.linalg.norm(agent - target) > 14
                and np.linalg.norm(box - target) > (14 if task == "push" else 6)
                and np.linalg.norm(agent - box) > 6):
            return target, agent, box
    raise RuntimeError("sampling failed")


@torch.no_grad()
def rollout(model, stats, vocab, task, seed, device, max_steps=E.T) -> tuple[bool, int]:
    """滚动时域（receding horizon）闭环：每步重取观测 → 预测块 → 只执行第 1 步。"""
    rng = np.random.default_rng(seed)
    target, agent, box = sample_start(task, rng)
    bg = int(rng.integers(18, 36))
    noise_seed = seed * 100003
    text = E.TEXTS[task][seed % len(E.TEXTS[task])]

    lang, lang_mask = encode_text(text, vocab)
    lang = torch.from_numpy(lang)[None].to(device)
    lang_mask = torch.from_numpy(lang_mask)[None].to(device)

    for t in range(max_steps):
        img = E.render_frame(agent, box, target, task, bg, noise_seed + t)
        state = (agent / (E.S / 2) - 1).astype(np.float32)   # 与训练同一状态约定
        x = torch.from_numpy(img).permute(2, 0, 1)[None].float().to(device) / 255.0
        pred = model(x, lang, lang_mask,
                     torch.from_numpy(state)[None].to(device))
        a = unnormalize(pred[0, 0].cpu().numpy(), stats)     # 执行块内第 1 步
        agent, box = E.env_step(agent, box, task, a)
        if E.is_success(agent, box, target, task):
            return True, t + 1
    return False, max_steps


def wilson_ci(p: float, n: int, z: float = 1.96) -> tuple[float, float]:
    """小样本成功率的 Wilson 95% 置信区间（docs/10 §2）。"""
    denom = 1 + z * z / n
    center = (p + z * z / (2 * n)) / denom
    half = z * np.sqrt((p * (1 - p) + z * z / (4 * n)) / n) / denom
    return center - half, center + half


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ckpt", default="runs/mini_vla/ckpt.pt")
    ap.add_argument("--assets", default=None, help="默认取 ckpt 所在目录")
    ap.add_argument("--n", type=int, default=40)
    ap.add_argument("--seed0", type=int, default=999_000)
    ap.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    args = ap.parse_args()
    assets = args.assets or str(pathlib.Path(args.ckpt).parent)

    model, cfg, stats, vocab, meta = load_model(args.ckpt, assets)
    model.to(args.device)
    print(f"ckpt={args.ckpt}  epoch={meta.get('epoch')}  "
          f"val_loss={meta.get('val_loss'):.4f}  n/任务={args.n}")
    for task in E.TASKS:
        res = [rollout(model, stats, vocab, task, args.seed0 + i, args.device)
               for i in range(args.n)]
        p = float(np.mean([r[0] for r in res]))
        avg_steps = float(np.mean([r[1] for r in res]))
        lo, hi = wilson_ci(p, args.n)
        print(f"task={task:5s}  success={p*100:5.1f}%  "
              f"Wilson95=[{lo*100:.1f}, {hi*100:.1f}]  avg_steps={avg_steps:.1f}")


if __name__ == "__main__":
    main()
