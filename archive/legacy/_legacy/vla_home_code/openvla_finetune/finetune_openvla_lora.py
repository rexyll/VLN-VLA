# -*- coding: utf-8 -*-
"""
finetune_openvla_lora.py —— OpenVLA 7B LoRA 微调参考实现（工程路径）

定位
    官方微调脚本的教学精简版，逐条对应 docs/04、docs/05 的工程要点。
    真实项目请以官方仓库为准（API 更新快）：
        离散 token 路线: https://github.com/openvla/openvla      (vla-scripts/finetune.py)
        OFT 连续头路线:  https://github.com/moojink/openvla-oft  (推理快 25~50x, 推荐)
    环境要求: 官方仓库环境 + flash-attn + peft；微调 1×24GB 起步。

与文档的对应
    docs/04 §3  数据: LeRobot/RLDS → (image, instruction, proprio, action)
    docs/04 §4  归一化: 分位数 q01/q99；统计量随模型归档（本脚本最后落盘）
    docs/05 §4  LoRA: r=16~64, attention+MLP, lr≈5e-4, bf16
    docs/05 §4.2 损失: 动作 token 交叉熵（本脚本）；OFT 路线为连续动作 L1
    docs/09     高频坑: 提示模板逐字符一致、动作 token 偏移、gripper 语义反转
"""
from __future__ import annotations

import argparse
import pathlib

import numpy as np
import torch
from PIL import Image
from torch.utils.data import DataLoader, Dataset

PROMPT_TEMPLATE = "In: What action should the robot take to {instruction}?\nOut:"
NUM_BINS = 256          # OpenVLA 惯例：每维 256 离散桶
ACTION_DIM = 7          # delta EEF: dx dy dz droll dpitch dyaw gripper（单位/坐标系必须成文！）


def quantile_normalize(a: np.ndarray, stats: dict) -> np.ndarray:
    """docs/04 §4：训练前归一化；部署端用同一份 stats 反归一化。"""
    q01, q99 = np.asarray(stats["q01"]), np.asarray(stats["q99"])
    a = np.clip(a, q01, q99)
    return 2 * (a - q01) / (q99 - q01 + 1e-8) - 1


def discretize(a_norm: np.ndarray) -> np.ndarray:
    """[-1,1] → 0..255 桶 id。官方实现在 prismatic 的 action_tokenizer.py，
    桶边界/偏移细节以其为准（docs/09：偏移写错 = 模型输出全错）。"""
    return np.clip(((a_norm + 1) / 2 * (NUM_BINS - 1)).round(), 0, NUM_BINS - 1).astype(np.int64)


class RobotDataset(Dataset):
    """把机器人数据适配成 VLA 样本。

    TODO(真实项目): 用 LeRobotDataset/RLDS 替换占位读取——
        image  : 相机帧 uint8 (H,W,3)，预处理分辨率交给 processor
        action : 与 docs 选型一致的 EEF delta (7,)
        instruction: 符合标注规范的语言指令 (docs/04 §5)
    """

    def __init__(self, data_dir: str, stats_path: str):
        self.stats = _read_json(stats_path)
        self.files = sorted(pathlib.Path(data_dir).glob("*.npz"))

    def __len__(self):
        return len(self.files)

    def __getitem__(self, i):
        d = np.load(self.files[i])
        return {"image": Image.fromarray(d["image"]),
                "prompt": PROMPT_TEMPLATE.format(instruction=str(d["instruction"])),
                "bins": discretize(quantile_normalize(d["action"].astype(np.float32),
                                                      self.stats))}


def _read_json(p):
    import json
    return json.loads(pathlib.Path(p).read_text("utf-8"))


def collate(batch, processor, tokenizer_len: int, pad_id: int) -> dict:
    """拼 [prompt + 动作 token + EOS]；labels 只监督动作段（语言/图像/pad 位 = -100）。"""
    enc = processor([b["prompt"] for b in batch],
                    [b["image"] for b in batch],
                    return_tensors="pt", padding="longest")
    rows_ids, rows_labels = [], []
    for row, b in enumerate(batch):
        prompt_ids = enc["input_ids"][row].tolist()
        # 动作 token id = tokenizer 词长 + 维度块偏移 ×256 + 桶 id（官方 ActionTokenizer 规则）
        act_ids = [tokenizer_len + k * NUM_BINS + int(v)
                   for k, v in enumerate(b["bins"])]
        rows_ids.append(prompt_ids + act_ids + [pad_id])
        rows_labels.append([-100] * len(prompt_ids) + act_ids + [-100])
    maxlen = max(len(r) for r in rows_ids)
    input_ids = torch.full((len(rows_ids), maxlen), pad_id, dtype=torch.long)
    labels = torch.full((len(rows_ids), maxlen), -100, dtype=torch.long)
    attn = torch.zeros((len(rows_ids), maxlen), dtype=torch.long)
    for i, (ids, labs) in enumerate(zip(rows_ids, rows_labels)):
        input_ids[i, :len(ids)] = torch.tensor(ids)
        labels[i, :len(labs)] = torch.tensor(labs)
        attn[i, :len(ids)] = 1
    return {"input_ids": input_ids, "attention_mask": attn,
            "pixel_values": enc["pixel_values"], "labels": labels}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default="openvla/openvla-7b")
    ap.add_argument("--data", required=True, help="样本目录（RobotDataset 占位实现）")
    ap.add_argument("--stats", required=True, help="norm_stats.json（q01/q99 每维）")
    ap.add_argument("--out", default="runs/openvla_lora")
    ap.add_argument("--epochs", type=int, default=20)
    ap.add_argument("--bs", type=int, default=8)
    ap.add_argument("--grad-accum", type=int, default=4)   # 有效 batch = bs × accum
    ap.add_argument("--lr", type=float, default=5e-4)
    ap.add_argument("--lora-r", type=int, default=32)
    args = ap.parse_args()

    from peft import LoraConfig, get_peft_model
    from transformers import AutoModelForVision2Seq, AutoProcessor

    processor = AutoProcessor.from_pretrained(args.model, trust_remote_code=True)
    model = AutoModelForVision2Seq.from_pretrained(
        args.model, torch_dtype=torch.bfloat16, trust_remote_code=True,
        low_cpu_mem_usage=True)
    tokenizer_len = len(processor.tokenizer)
    pad_id = processor.tokenizer.pad_token_id or 0

    lcfg = LoraConfig(r=args.lora_r, lora_alpha=args.lora_r * 2, lora_dropout=0.0,
                      target_modules=["q_proj", "k_proj", "v_proj", "o_proj",
                                      "gate_proj", "up_proj", "down_proj"])
    model = get_peft_model(model, lcfg)
    model.print_trainable_parameters()
    model.cuda()

    ds = RobotDataset(args.data, args.stats)
    loader = DataLoader(ds, batch_size=args.bs, shuffle=True,
                        collate_fn=lambda b: collate(b, processor, tokenizer_len, pad_id))
    params = [p for p in model.parameters() if p.requires_grad]
    opt = torch.optim.AdamW(params, lr=args.lr)

    model.train()
    for epoch in range(args.epochs):
        opt.zero_grad(set_to_none=True)
        for step, batch in enumerate(loader):
            batch = {k: v.cuda(non_blocking=True) for k, v in batch.items()}
            loss = model(**batch).loss / args.grad_accum   # CE 只来自动作 token 位
            loss.backward()
            if (step + 1) % args.grad_accum == 0:
                torch.nn.utils.clip_grad_norm_(params, 1.0)
                opt.step()
                opt.zero_grad(set_to_none=True)
        print(f"epoch {epoch+1}/{args.epochs}  last_loss={loss.item()*args.grad_accum:.4f}")

    # 部署四件套（docs/06 §7）：adapter + processor + 归一化统计 + 提示模板
    out = pathlib.Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    model.save_pretrained(str(out))
    processor.save_pretrained(str(out))
    (out / "norm_stats.json").write_text(pathlib.Path(args.stats).read_text("utf-8"), "utf-8")
    (out / "prompt_template.txt").write_text(PROMPT_TEMPLATE, "utf-8")
    print(f"saved -> {out}")


if __name__ == "__main__":
    main()
