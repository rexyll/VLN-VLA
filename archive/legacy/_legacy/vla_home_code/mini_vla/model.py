# -*- coding: utf-8 -*-
"""
model.py —— MiniVLA：视觉编码器 + 语言嵌入 + 单流 Transformer + 动作块回归头

教学定位
    用 ~2M 参数演示 VLA 的核心结构：三模态 token 拼接 → Transformer 融合 → 动作块输出。
    与生产级模型的差异（见 docs/01 §2.3、docs/02）：
      - 骨干：生产用 LLM/VLM（PaliGemma、Qwen-VL 等，带互联网先验）；本模型从零训练
      - 动作头：生产用离散 token / flow matching；本模型用连续回归 + tanh
      - 本模型用全注意力（策略不需要因果掩码），生产 LLM 骨干为因果注意力
"""
from __future__ import annotations

from dataclasses import dataclass

import torch
import torch.nn as nn


@dataclass
class MiniVLAConfig:
    vocab_size: int = 64
    max_lang_len: int = 12
    img_size: int = 64
    patch_size: int = 8
    in_channels: int = 3
    dim: int = 256
    depth: int = 4
    heads: int = 4
    mlp_ratio: int = 4
    horizon: int = 8          # 动作块长度 H（docs/01 §3）
    action_dim: int = 2
    state_dim: int = 2


class PatchEmbed(nn.Module):
    """图像 → 8x8 patch 序列 + 可学习位置编码。"""

    def __init__(self, cfg: MiniVLAConfig):
        super().__init__()
        self.proj = nn.Conv2d(cfg.in_channels, cfg.dim, cfg.patch_size, cfg.patch_size)
        n_tokens = (cfg.img_size // cfg.patch_size) ** 2
        self.pos = nn.Parameter(torch.zeros(1, n_tokens, cfg.dim))
        nn.init.trunc_normal_(self.pos, std=0.02)

    def forward(self, x):  # (B,3,64,64) -> (B,64,dim)
        return self.proj(x).flatten(2).transpose(1, 2) + self.pos


class Block(nn.Module):
    """Pre-LN Transformer 块（全注意力，key_padding_mask 屏蔽语言 pad）。"""

    def __init__(self, cfg: MiniVLAConfig):
        super().__init__()
        self.n1 = nn.LayerNorm(cfg.dim)
        self.attn = nn.MultiheadAttention(cfg.dim, cfg.heads, batch_first=True)
        self.n2 = nn.LayerNorm(cfg.dim)
        hidden = cfg.dim * cfg.mlp_ratio
        self.mlp = nn.Sequential(nn.Linear(cfg.dim, hidden), nn.GELU(),
                                 nn.Linear(hidden, cfg.dim))

    def forward(self, x, key_padding_mask=None):
        h = self.n1(x)
        x = x + self.attn(h, h, h, key_padding_mask=key_padding_mask,
                          need_weights=False)[0]
        return x + self.mlp(self.n2(x))


class MiniVLA(nn.Module):
    """输入 token 流 = [语言 xL][视觉 x64][本体感知 x1][ACT 查询 x1]。

    取 ACT 查询位的输出进动作头，回归未来 H 步动作块（tanh 映射到 [-1,1]，
    与归一化约定一致，docs/04 §4）。
    """

    def __init__(self, cfg: MiniVLAConfig):
        super().__init__()
        self.cfg = cfg
        self.vision = PatchEmbed(cfg)
        self.lang = nn.Embedding(cfg.vocab_size, cfg.dim)
        self.lang_pos = nn.Parameter(torch.zeros(1, cfg.max_lang_len, cfg.dim))
        nn.init.trunc_normal_(self.lang_pos, std=0.02)
        self.state_proj = nn.Linear(cfg.state_dim, cfg.dim)
        self.act_query = nn.Parameter(torch.zeros(1, 1, cfg.dim))
        nn.init.trunc_normal_(self.act_query, std=0.02)
        self.blocks = nn.ModuleList(Block(cfg) for _ in range(cfg.depth))
        self.norm = nn.LayerNorm(cfg.dim)
        self.action_head = nn.Sequential(
            nn.Linear(cfg.dim + 4, cfg.dim), nn.GELU(),
            nn.Linear(cfg.dim, cfg.horizon * cfg.action_dim),
        )
        # 训练期辅助头：预测目标位置（归一化坐标）。密集监督帮助视觉通路快速学会定位；
        # 推理时不使用（ZR-0 式 dense supervision 的极简版）。
        self.aux_head = nn.Linear(cfg.dim, 2)

    def forward(self, image, lang, lang_mask, state, return_aux: bool = False):
        B = image.shape[0]
        v = self.vision(image)                                   # (B,64,D)
        l = self.lang(lang) + self.lang_pos[:, : lang.shape[1]]  # (B,L,D)
        s = self.state_proj(state).unsqueeze(1)                  # (B,1,D)
        q = self.act_query.expand(B, 1, -1)                      # (B,1,D)
        x = torch.cat([l, v, s, q], dim=1)
        # key_padding_mask: True=忽略。仅语言 pad 位被屏蔽，其余 token 全保留。
        pad = ~lang_mask.bool()
        kpm = torch.cat([pad, torch.zeros(B, v.shape[1] + 2, dtype=torch.bool,
                                          device=image.device)], dim=1)
        for blk in self.blocks:
            x = blk(x, kpm)
        out = self.norm(x[:, -1])                                # 末位 = ACT 查询
        # 线性输出（不经过 tanh）：避免饱和区梯度消失，动作范围由 unnormalize 端裁剪保证
        aux = self.aux_head(out)                                 # (B,2) 目标位置估计
        # 把目标估计与显式误差信号(aux-state)接入动作头：给动作头 P 控制器同款信息，
        # 消除“靠近目标该停还是该继续”的监督歧义（末端停止行为泛化 bug 的修复）
        feat = torch.cat([out, aux, aux - state], dim=-1)
        chunk = self.action_head(feat).view(
            B, self.cfg.horizon, self.cfg.action_dim)
        if return_aux:
            return chunk, aux
        return chunk

    def num_params(self) -> int:
        return sum(p.numel() for p in self.parameters())
