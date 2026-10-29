# -*- coding: utf-8 -*-
"""train_gz.py —— 在 Gazebo 域内数据上训练 MiniVLA（数据飞轮的"训练"半环）

用法: python3 train_gz.py --data gz_data.npz --out runs/gz_model --epochs 15
前置: collect_gazebo_data.py 已产出 gz_data.npz
"""
from __future__ import annotations

import argparse
import math
import pathlib
import sys
import time

import numpy as np
import torch
from torch.utils.data import DataLoader

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "mini_vla"))
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from gz_dataset import GzVLADataset  # noqa: E402
from model import MiniVLA, MiniVLAConfig  # noqa: E402
from train import AUX_W, evaluate_loss, masked_l1  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default="gz_data.npz")
    ap.add_argument("--out", default="runs/gz_model")
    ap.add_argument("--epochs", type=int, default=15)
    ap.add_argument("--bs", type=int, default=128)
    ap.add_argument("--lr", type=float, default=3e-4)
    ap.add_argument("--warmup", type=int, default=50)
    ap.add_argument("--H", type=int, default=8)
    ap.add_argument("--seed", type=int, default=42)
    ap.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    ap.add_argument("--init", default=None,
                    help="从已有 ckpt 热启动（合成域预训练→仿真域微调）")
    args = ap.parse_args()

    torch.manual_seed(args.seed)
    np.random.seed(args.seed)
    out = pathlib.Path(args.out)
    out.mkdir(parents=True, exist_ok=True)

    train_ds = GzVLADataset(args.data, "train", H=args.H)
    train_ds.save_assets(str(out))
    val_ds = GzVLADataset(args.data, "val", H=args.H,
                          stats_path=str(out / "norm_stats.json"),
                          vocab_path=str(out / "vocab.json"))
    train_loader = DataLoader(train_ds, batch_size=args.bs, shuffle=True, drop_last=True)
    val_loader = DataLoader(val_ds, batch_size=args.bs, shuffle=False)
    print(f"train={len(train_ds)}  val={len(val_ds)}  device={args.device}", flush=True)

    cfg = MiniVLAConfig(vocab_size=len(train_ds.vocab), horizon=args.H)
    model = MiniVLA(cfg).to(args.device)
    if args.init:
        ck = torch.load(args.init, map_location="cpu", weights_only=False)
        own = model.state_dict()
        filtered = {k: v for k, v in ck["model"].items()
                    if k in own and own[k].shape == v.shape}
        skipped = sorted(set(ck["model"]) - set(filtered))
        model.load_state_dict(filtered, strict=False)
        print(f"init from {args.init} (epoch {ck.get('epoch')}, val {ck.get('val_loss')})  "
              f"loaded={len(filtered)} shape跳过={skipped}", flush=True)
    opt = torch.optim.AdamW(model.parameters(), lr=args.lr, weight_decay=0.0)
    total_steps = max(1, args.epochs * len(train_loader))

    def lr_lambda(step):
        if step < args.warmup:
            return step / max(1, args.warmup)
        p = (step - args.warmup) / max(1, total_steps - args.warmup)
        return 0.5 * (1.0 + math.cos(math.pi * min(1.0, p)))

    sched = torch.optim.lr_scheduler.LambdaLR(opt, lr_lambda)

    best = float("inf")
    for epoch in range(1, args.epochs + 1):
        t0 = time.time()
        run, nb = 0.0, 0
        for batch in train_loader:
            pred, aux = model(batch["image"].to(args.device), batch["lang"].to(args.device),
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
              f"({time.time()-t0:.1f}s){mark}", flush=True)
    print(f"done. best val={best:.4f} -> {out/'ckpt.pt'}")


if __name__ == "__main__":
    main()
