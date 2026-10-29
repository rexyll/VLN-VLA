#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""debug_diag.py —— 临时诊断：训练与 rollout 之间为何脱节（非交付代码）"""
import sys, pathlib
import numpy as np
import torch

sys.path.insert(0, str(pathlib.Path("VLA知识库/code/mini_vla").resolve()))
from dataset import MiniVLADataset, unnormalize
from evaluate import load_model
import env_synth as E

CKPT = "VLA知识库/code/mini_vla/runs/mini_vla_v3/ckpt.pt"
ASSETS = "VLA知识库/code/mini_vla/runs/mini_vla_v3"
DATA = "VLA知识库/code/mini_vla/data_synth.npz"

model, cfg, stats, vocab, meta = load_model(CKPT, ASSETS)
ds = MiniVLADataset(DATA, "train", H=8,
                    stats_path=ASSETS + "/norm_stats.json",
                    vocab_path=ASSETS + "/vocab.json")
d = ds._d
ep_pos = {int(e): k for k, e in enumerate(ds.ep_idx)}

# ---------- 1) 训练样本上的单步预测精度 ----------
errs, agree = [], []
with torch.no_grad():
    for i in range(0, len(ds), max(1, len(ds)//300)):
        s = ds[i]
        pred = model(s["image"][None], s["lang"][None], s["lang_mask"][None], s["state"][None])
        p0 = unnormalize(pred[0, 0].numpy(), stats)
        t0 = unnormalize(s["chunk"][0].numpy(), stats)
        mask = float(s["chunk_mask"][0])
        errs.append(np.abs(p0 - t0).mean())
        if np.linalg.norm(t0) > 5:
            cos = float(np.dot(p0, t0) / (np.linalg.norm(p0) * np.linalg.norm(t0) + 1e-9))
            agree.append(cos)
print(f"[1] 单步: mean|err|={np.mean(errs):.2f}px/s  方向余弦均值={np.mean(agree):.3f}  n={len(errs)}")

# ---------- 2) 训练回合的离线重放（数据同分布输入, 闭环执行） ----------
@torch.no_grad()
def replay(e_id):
    k = ep_pos[e_id]
    task = str(d["task"][e_id]); target = d["target"][e_id]
    seed = int(d["seed"][e_id]); bg = int(d["bg"][e_id])
    text = str(d["text"][e_id])
    agent = d["agent"][e_id, 0].astype(np.float64).copy()
    box = d["box"][e_id, 0].astype(np.float64).copy()
    lang, lang_mask = None, None
    from dataset import encode_text
    lang, lang_mask = encode_text(text, vocab)
    lang_t = torch.from_numpy(lang)[None]; mask_t = torch.from_numpy(lang_mask)[None]
    for t in range(E.T):
        img = E.render_frame(agent, box, target, task, bg, seed * 100003 + t)
        state = (agent / (E.S / 2) - 1).astype(np.float32)
        x = torch.from_numpy(img).permute(2, 0, 1)[None].float() / 255.0
        pred = model(x, lang_t, mask_t, torch.from_numpy(state)[None])
        a = unnormalize(pred[0, 0].numpy(), stats)
        agent, box = E.env_step(agent, box, task, a)
        if E.is_success(agent, box, target, task):
            return True, t + 1
    return False, E.T

ok = [replay(int(e)) for e in list(ep_pos)[:10]]
print(f"[2] 训练回合闭环重放: 成功 {sum(1 for s,_ in ok if s)}/10  步数 {[s for _,s in ok]}")

# ---------- 3) 看看模型到底输出什么（一个训练样本） ----------
s = ds[ep_pos[0] * E.T + 0]
with torch.no_grad():
    pred = model(s["image"][None], s["lang"][None], s["lang_mask"][None], s["state"][None])
print(f"[3] 样本: 目标chunk[0]={unnormalize(s['chunk'][0].numpy(), stats).round(1).tolist()}  "
      f"预测chunk[0]={unnormalize(pred[0,0].numpy(), stats).round(1).tolist()}")
print(f"    state={s['state'].numpy().round(2).tolist()}  text={str(d['text'][ds.ep_idx[0]])!r}")
