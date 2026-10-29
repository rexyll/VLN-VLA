# -*- coding: utf-8 -*-
"""
dataset.py —— 合成数据的 PyTorch 数据集：按需渲染 + 动作块 + 归一化 + 词表

工程要点（对应 docs/04-数据体系）
    1) 只存标量轨迹，图像按 (episode, t) 确定性渲染 —— 数据文件几 MB，可复现。
    2) 动作块 chunk：目标为未来 H 步动作，episode 末端补零并给 mask。
    3) 归一化统计只用训练集计算，持久化为 norm_stats.json，随模型部署。
    4) 词表与填充约定持久化为 vocab.json，训练/部署共用。
"""
from __future__ import annotations

import json
import pathlib

import numpy as np
import torch
from torch.utils.data import Dataset

import env_synth as E

MAX_LANG_LEN = 12


# ---------------------------------------------------------------- 词表
def build_vocab(texts) -> dict:
    vocab = {"<pad>": 0}
    for t in texts:
        for w in t.split():
            vocab.setdefault(w, len(vocab))
    vocab["<unk>"] = len(vocab)
    return vocab


def encode_text(text: str, vocab: dict, max_len: int = MAX_LANG_LEN):
    ids = [vocab.get(w, vocab["<unk>"]) for w in text.split()][:max_len]
    mask = [1] * len(ids)
    n_pad = max_len - len(ids)          # 先算补齐量，再分别补齐（顺序反了会出 shape 不一致 bug）
    ids += [0] * n_pad
    mask += [0] * n_pad
    return np.array(ids, np.int64), np.array(mask, np.int64)


# ---------------------------------------------------------------- 归一化
def fit_norm_stats(actions: np.ndarray) -> dict:
    """分位数归一化统计（OXE/RT 惯例）：对离群值稳健。"""
    q01 = np.quantile(actions, 0.01, axis=0)
    q99 = np.quantile(actions, 0.99, axis=0)
    return {"q01": q01.tolist(), "q99": q99.tolist(), "dim": int(actions.shape[1])}


def normalize(a: np.ndarray, stats: dict) -> np.ndarray:
    q01, q99 = np.asarray(stats["q01"]), np.asarray(stats["q99"])
    a = np.clip(a, q01, q99)
    return (2 * (a - q01) / (q99 - q01 + 1e-8) - 1).astype(np.float32)


def unnormalize(a, stats: dict) -> np.ndarray:
    q01, q99 = np.asarray(stats["q01"]), np.asarray(stats["q99"])
    raw = q01 + (np.asarray(a) + 1) / 2 * (q99 - q01)
    return np.clip(raw, q01, q99)


# ---------------------------------------------------------------- 数据集
class MiniVLADataset(Dataset):
    """索引空间 = episode × 时间步。train 划分与统计量计算都必须只看训练集。"""

    def __init__(self, npz_path: str, split: str = "train", H: int = 8,
                 stats_path: str | None = None, vocab_path: str | None = None,
                 val_ratio: float = 0.15, split_seed: int = 0):
        d = np.load(npz_path, allow_pickle=True)
        self._d = {k: d[k] for k in d.files}
        self.T = self._d["agent"].shape[1]
        self.H = H
        # 有效帧长（成功后截断零动作长尾；无 tlen 的旧数据按全长处理）
        self.tlen = self._d.get("tlen", np.full(self._d["agent"].shape[0], self.T, np.int32))

        rng = np.random.default_rng(split_seed)
        n_ep = self._d["agent"].shape[0]
        perm = rng.permutation(n_ep)
        n_val = int(round(n_ep * val_ratio))
        idx = perm[:n_val] if split == "val" else perm[n_val:]
        self.ep_idx = np.sort(idx)
        self.index = [(e, t) for e in self.ep_idx for t in range(int(self.tlen[e]))]

        # 词表：训练集重建或从文件加载（部署端永远加载文件）
        if vocab_path and pathlib.Path(vocab_path).exists():
            self.vocab = json.loads(pathlib.Path(vocab_path).read_text("utf-8"))
        else:
            train_texts = self._d["text"][perm[n_val:]]
            self.vocab = build_vocab(train_texts)

        # 归一化统计：只用训练集动作计算；文件存在则加载（保证一致性）
        if stats_path and pathlib.Path(stats_path).exists():
            self.stats = json.loads(pathlib.Path(stats_path).read_text("utf-8"))
        elif split == "train":
            train_eps = perm[n_val:]
            self.stats = fit_norm_stats(self._d["actions"][train_eps].reshape(-1, 2))
        else:
            raise RuntimeError("val split 必须提供已保存的 stats_path（防止统计泄漏）")

    # -------------------------------------------------- 保存/加载资产
    def save_assets(self, out_dir: str) -> None:
        p = pathlib.Path(out_dir); p.mkdir(parents=True, exist_ok=True)
        (p / "vocab.json").write_text(json.dumps(self.vocab, ensure_ascii=False, indent=1), "utf-8")
        (p / "norm_stats.json").write_text(json.dumps(self.stats, indent=1), "utf-8")

    # -------------------------------------------------- 取样
    def __len__(self):
        return len(self.index)

    def __getitem__(self, i: int):
        e, t = self.index[i]
        d = self._d
        seed = int(d["seed"][e]); bg = int(d["bg"][e]); task = str(d["task"][e])

        img = E.render_frame(d["agent"][e, t], d["box"][e, t], d["target"][e],
                             task, bg, seed * 100003 + t)
        state = (d["agent"][e, t] / (E.S / 2) - 1).astype(np.float32)  # → [-1,1]

        lang, lang_mask = encode_text(str(d["text"][e]), self.vocab)

        # 动作块目标：未来 H 步；越界补零并 mask=0（成功后的 0 动作保留 mask=1，学习"停下"）
        actions = d["actions"][e]
        chunk = np.zeros((self.H, 2), np.float32)
        m = np.zeros(self.H, np.float32)
        n = min(self.H, int(self.tlen[e]) - t)
        chunk[:n] = normalize(actions[t:t + n], self.stats)
        m[:n] = 1.0

        return {
            "image": torch.from_numpy(img.copy()).permute(2, 0, 1).float() / 255.0,
            "state": torch.from_numpy(state),
            "lang": torch.from_numpy(lang),
            "lang_mask": torch.from_numpy(lang_mask),
            "chunk": torch.from_numpy(chunk),
            "chunk_mask": torch.from_numpy(m),
            # 辅助监督目标：任务目标位置（归一化到 [-1,1]，与 state 同约定）
            "target": torch.from_numpy(
                (d["target"][e] / (E.S / 2) - 1).astype(np.float32)),
            "task": task,
        }
