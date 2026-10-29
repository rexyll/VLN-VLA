# -*- coding: utf-8 -*-
"""merge_npz.py —— 合并多个同 schema 的演示数据集（帧长对齐到最大值）
用法: python3 merge_npz.py <out.npz> <in1.npz> [in2.npz ...]
"""
import sys

import numpy as np

out, ins = sys.argv[1], sys.argv[2:]
print("argv:", sys.argv, flush=True)
sets = [dict(np.load(p, allow_pickle=True)) for p in ins]
for p, s in zip(ins, sets):
    print(f"loaded {p}: E={s['agent'].shape[0]} T={s['agent'].shape[1]}", flush=True)
Tmax = max(int(s["tlen"].sum() and s["agent"].shape[1]) for s in sets)
TIME_KEYS = {"images", "agent", "box", "actions"}   # 仅这些数组的第二维是时间
keys = sets[0].keys()
merged = {}
for k in keys:
    parts = []
    for s in sets:
        a = s[k]
        if k in TIME_KEYS and a.ndim >= 2 and a.shape[1] < Tmax:
            # pad_width 按轴序: [(0,0)=episode 轴, (0,Δ)=时间轴, 其余轴不动]
            pad = [(0, 0), (0, Tmax - a.shape[1])] + [(0, 0)] * (a.ndim - 2)
            a = np.pad(a, pad)
        parts.append(a)
    try:
        merged[k] = np.concatenate(parts, axis=0)
    except ValueError as exc:
        print(f"[merge-fail] key={k!r} shapes={[p.shape for p in parts]}")
        raise
    print(f"merged {k}: {merged[k].shape}")
np.savez_compressed(out, **merged)
n = merged["tlen"].shape[0]
print(f"saved {out}: episodes={n} frames={int(merged['tlen'].sum())} "
      f"tasks={{k: int(v) for k, v in zip(*np.unique(merged['task'], return_counts=True))}}")
