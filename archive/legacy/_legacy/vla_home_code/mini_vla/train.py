# -*- coding: utf-8 -*-
"""
train.py —— MiniVLA 训练脚本（docs/05-训练流程 的可运行对应）

用法:
    python train.py --data data_synth.npz --out runs/mini_vla --epochs 40

要点
    - 损失 = 掩码 L1（动作块 padding 不计损失，docs/05 §4.2）
    - 词表 + 归一化统计在训练开始时随 checkpoint 归档（部署四件套之二，docs/04 §4）
    - val 划分按 episode、统计量只用训练集（防泄漏，docs/04 §7）
    - 离线 loss 只是初筛，最终以 evaluate.py 的 rollout 成功率定版（docs/05 §5）
"""
from __future__ import annotations

import argparse
import math
import pathlib
import sys
import time

# 保证在任意 Python 环境（含嵌入式/隔离模式）下都能导入同目录模块
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))

import numpy as np
import torch
from torch.utils.data import DataLoader

from dataset import MiniVLADataset
from model import MiniVLA, MiniVLAConfig


def masked_l1(pred, target, mask):
    """pred/target: (B,H,d)，mask: (B,H)∈{0,1}。对块内有效步取 L1 再平均。"""
    err = (pred - target).abs().mean(-1)          # (B,H)
    return (err * mask).sum() / mask.sum().clamp(min=1.0)


AUX_W = 1.0   # 辅助定位损失权重：密集监督加速视觉通路收敛（推理不使用）


@torch.no_grad()
def evaluate_loss(model, loader, device):
    model.eval()
    tot, n = 0.0, 0
    for batch in loader:
        pred = model(batch["image"].to(device), batch["lang"].to(device),
                     batch["lang_mask"].to(device), batch["state"].to(device))
        loss = masked_l1(pred, batch["chunk"].to(device),
                         batch["chunk_mask"].to(device))
        bs = batch["chunk"].shape[0]
        tot += loss.item() * bs
        n += bs
    model.train()
    return tot / max(1, n)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default="data_synth.npz")
    ap.add_argument("--out", default="runs/mini_vla")
    ap.add_argument("--epochs", type=int, default=40)
    ap.add_argument("--bs", type=int, default=256)
    ap.add_argument("--lr", type=float, default=3e-4)
    ap.add_argument("--warmup", type=int, default=100)
    ap.add_argument("--H", type=int, default=8)
    ap.add_argument("--workers", type=int, default=0)   # Windows 默认 0；Linux 可调大
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    args = ap.parse_args()

    torch.manual_seed(args.seed)
    np.random.seed(args.seed)
    out = pathlib.Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    # 1) 数据：先建 train（算统计量）→ 归档资产 → 再建 val（强制复用同一份统计量）
    train_ds = MiniVLADataset(args.data, "train", H=args.H)
    train_ds.save_assets(str(out))                      # vocab.json + norm_stats.json
    val_ds = MiniVLADataset(args.data, "val", H=args.H,
                            stats_path=str(out / "norm_stats.json"),
                            vocab_path=str(out / "vocab.json"))
    train_loader = DataLoader(train_ds, batch_size=args.bs, shuffle=True,
                              num_workers=args.workers, drop_last=True)
    val_loader = DataLoader(val_ds, batch_size=args.bs, shuffle=False,
                            num_workers=args.workers)

    # 2) 模型与优化
    cfg = MiniVLAConfig(vocab_size=len(train_ds.vocab), horizon=args.H)
    model = MiniVLA(cfg).to(args.device)
    print(f"params={model.num_params()/1e6:.2f}M  train={len(train_ds)} "
          f"val={len(val_ds)}  device={args.device}")

    opt = torch.optim.AdamW(model.parameters(), lr=args.lr, weight_decay=0.0)
    total_steps = max(1, args.epochs * len(train_loader))

    def lr_lambda(step):
        if step < args.warmup:
            return step / max(1, args.warmup)
        p = (step - args.warmup) / max(1, total_steps - args.warmup)
        return 0.5 * (1.0 + math.cos(math.pi * min(1.0, p)))

    sched = torch.optim.lr_scheduler.LambdaLR(opt, lr_lambda)

    # 3) 训练循环
    best = float("inf")
    for epoch in range(1, args.epochs + 1):
        t0 = time.time()
        run, nb = 0.0, 0
        for batch in train_loader:
            pred, aux = model(batch["image"].to(args.device),
                              batch["lang"].to(args.device),
                              batch["lang_mask"].to(args.device),
                              batch["state"].to(args.device), return_aux=True)
            loss = masked_l1(pred, batch["chunk"].to(args.device),
                             batch["chunk_mask"].to(args.device))
            loss = loss + AUX_W * (aux - batch["target"].to(args.device)).abs().mean()
            opt.zero_grad(set_to_none=True)
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), 1.0)
            opt.step()
            sched.step()
            run += loss.item()
            nb += 1
        vl = evaluate_loss(model, val_loader, args.device)
        mark = ""
        if vl < best:
            best = vl
            mark = "  *best*"
            torch.save({"model": model.state_dict(), "config": cfg.__dict__,
                        "val_loss": vl, "epoch": epoch, "args": vars(args)},
                       out / "ckpt.pt")
        print(f"epoch {epoch:03d}  train {run/max(1,nb):.4f}  val {vl:.4f}  "
              f"({time.time()-t0:.1f}s){mark}")
    print(f"done. best val={best:.4f} -> {out/'ckpt.pt'}")


if __name__ == "__main__":
    main()
