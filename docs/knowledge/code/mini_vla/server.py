# -*- coding: utf-8 -*-
"""
server.py —— 推理服务：POST /act（图像+指令+本体感知）→ 动作块

对应 docs/06 §4：控制进程与推理进程解耦。robot_loop.py 是本服务的客户端。
接口契约（生产必须成文并带版本号）：
    入参 image_b64(PNG/JPEG) + instruction(str) + state(像素坐标 [x,y])
    出参 actions((H,2) 未归一化 px/s) + horizon + model_version

用法:
    python server.py --ckpt runs/mini_vla/ckpt.pt --port 8000
"""
from __future__ import annotations

import argparse
import base64
import io
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))  # 任意环境可导入同目录模块

import numpy as np
import torch
from fastapi import FastAPI
from PIL import Image
from pydantic import BaseModel

import env_synth as E
from dataset import encode_text, unnormalize
from evaluate import load_model


class ActRequest(BaseModel):
    image_b64: str
    instruction: str
    state: list[float]


class ActResponse(BaseModel):
    actions: list
    horizon: int
    model_version: str


def build_app(ckpt_path: str) -> FastAPI:
    device = "cuda" if torch.cuda.is_available() else "cpu"
    model, cfg, stats, vocab, meta = load_model(
        ckpt_path, str(__import__("pathlib").Path(ckpt_path).parent))
    model.to(device).eval()
    version = f"mini_vla@epoch{meta.get('epoch')}"

    app = FastAPI(title="MiniVLA inference server")

    @app.get("/health")
    def health():
        return {"ok": True, "version": version}

    @app.post("/act", response_model=ActResponse)
    def act(req: ActRequest):
        raw = base64.b64decode(req.image_b64)
        img = np.asarray(Image.open(io.BytesIO(raw)).convert("RGB"))
        # 教学约定：分辨率必须与训练一致(64x64)；生产中在此做与训练相同的 resize/裁剪
        x = torch.from_numpy(img.copy()).permute(2, 0, 1)[None].float().to(device) / 255.0

        lang, lang_mask = encode_text(req.instruction, vocab)
        lang = torch.from_numpy(lang)[None].to(device)
        lang_mask = torch.from_numpy(lang_mask)[None].to(device)

        # 状态归一化与训练一致：像素坐标 → [-1,1]（docs/09：约定漂移是最隐蔽的坑）
        state = np.asarray(req.state, dtype=np.float32) / (E.S / 2) - 1.0
        state_t = torch.from_numpy(state)[None].to(device)

        with torch.no_grad():
            chunk = model(x, lang, lang_mask, state_t)[0].cpu().numpy()
        actions = unnormalize(chunk, stats)          # 反归一化用同一份统计量！
        return ActResponse(actions=actions.tolist(),
                           horizon=int(actions.shape[0]),
                           model_version=version)

    return app


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--ckpt", default="runs/mini_vla/ckpt.pt")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8000)
    args = ap.parse_args()
    import uvicorn
    uvicorn.run(build_app(args.ckpt), host=args.host, port=args.port)
