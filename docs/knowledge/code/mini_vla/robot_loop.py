# -*- coding: utf-8 -*-
"""
robot_loop.py —— 机器人侧控制回路：观测 → 请求动作块 → 安全层 → 执行

对应 docs/06 §4-§5。DummyRobot 用合成环境扮演机器人；
真实机器人 = 只替换 DummyRobot 的实现（相机 SDK + 控制器 SDK），其余代码不动。

用法:
    python robot_loop.py --mode local  --ckpt runs/mini_vla/ckpt.pt --task reach
    python robot_loop.py --mode server --api http://127.0.0.1:8000 --task push
"""
from __future__ import annotations

import argparse
import base64
import io
import pathlib
import sys
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))  # 任意环境可导入同目录模块

import numpy as np

import env_synth as E

try:                      # 本地推理需要 torch；纯客户端(server 模式)可以不装
    import torch
except ImportError:
    torch = None


# ---------------------------------------------------------------- 机器人接口
class DummyRobot:
    """合成环境机器人。替换点：get_obs/step 换成相机与控制器 SDK。

    生产中的 get_obs 还应返回时间戳，并做观测-动作因果对齐（docs/09 §4）。
    """

    def __init__(self, task: str, seed: int):
        self.task = task
        rng = np.random.default_rng(seed)
        for _ in range(50):
            self.target = rng.uniform(10, E.S - 11, 2)
            self.agent = rng.uniform(6, E.S - 7, 2)
            self.box = rng.uniform(8, E.S - 9, 2)
            if (np.linalg.norm(self.agent - self.target) > 14
                    and np.linalg.norm(self.box - self.target)
                    > (14 if task == "push" else 6)
                    and np.linalg.norm(self.agent - self.box) > 6):
                break
        self.bg = int(rng.integers(18, 36))
        self.noise_seed = seed * 100003
        self.instruction = E.TEXTS[task][seed % len(E.TEXTS[task])]
        self.t = 0

    def get_obs(self) -> dict:
        img = E.render_frame(self.agent, self.box, self.target, self.task,
                             self.bg, self.noise_seed + self.t)
        return {"image": img, "state": self.agent.astype(np.float32),
                "instruction": self.instruction}

    def step(self, vel) -> bool:
        """下发速度指令一个控制周期，返回任务是否成功。"""
        self.agent, self.box = E.env_step(self.agent, self.box, self.task, vel)
        self.t += 1
        return E.is_success(self.agent, self.box, self.target, self.task)


# ---------------------------------------------------------------- 安全层
class SafetyLayer:
    """独立于模型的安全层（docs/06 §5 的精简教学版）。

    生产系统还需补充：关节限位、碰撞监测、急停链路、分级看门狗、
    不确定性降级（多采样分歧过大→减速）、人机共处区功率/力限制。
    """

    def __init__(self, workspace=(8.0, 56.0), max_speed: float = E.MAX_SPEED):
        self.lo, self.hi = workspace
        self.max_speed = max_speed

    def check(self, vel, state):
        """返回 (通过的动作, None) 或 (None, 拒绝原因)。任何可疑输入都拒绝。"""
        vel = np.asarray(vel, dtype=np.float64)
        state = np.asarray(state, dtype=np.float64)
        if not (np.all(np.isfinite(vel)) and np.all(np.isfinite(state))):
            return None, "non-finite action/observation"
        if np.abs(vel).max() > self.max_speed:
            vel = np.clip(vel, -self.max_speed, self.max_speed)   # 限速
        # 软限位：本合成环境内部已裁剪位置；真实机器人此处做工作空间盒校验/裁剪
        return vel, None


# ---------------------------------------------------------------- 策略端点
class LocalPolicy:
    """进程内推理（教学/调试用）。"""

    def __init__(self, ckpt: str, device=None):
        if torch is None:
            raise RuntimeError("local 模式需要安装 torch")
        from dataset import encode_text, unnormalize
        from evaluate import load_model
        self.encode_text, self.unnormalize = encode_text, unnormalize
        self.device = device or ("cuda" if torch.cuda.is_available() else "cpu")
        self.model, _cfg, self.stats, self.vocab, _meta = load_model(
            ckpt, str(pathlib.Path(ckpt).parent))
        self.model.to(self.device).eval()

    def act(self, obs: dict) -> np.ndarray:
        x = torch.from_numpy(obs["image"].copy()).permute(2, 0, 1)[None].float() / 255.0
        lang, mask = self.encode_text(obs["instruction"], self.vocab)
        lang = torch.from_numpy(lang)[None]
        m = torch.from_numpy(mask)[None]
        state = torch.from_numpy(obs["state"].astype(np.float32) / (E.S / 2) - 1.0)[None]
        with torch.no_grad():
            chunk = self.model(x.to(self.device), lang.to(self.device),
                               m.to(self.device), state.to(self.device))
        return self.unnormalize(chunk[0].cpu().numpy(), self.stats)


class ServerPolicy:
    """远程推理服务客户端（生产架构，docs/06 §4）。"""

    def __init__(self, api: str, timeout: float = 1.0):
        import requests
        from PIL import Image
        self._Image, self._requests = Image, requests
        self.api, self.timeout = api.rstrip("/"), timeout

    def act(self, obs: dict) -> np.ndarray:
        buf = io.BytesIO()
        self._Image.fromarray(obs["image"]).save(buf, format="PNG")
        r = self._requests.post(
            f"{self.api}/act",
            json={"image_b64": base64.b64encode(buf.getvalue()).decode(),
                  "instruction": obs["instruction"],
                  "state": obs["state"].tolist()},
            timeout=self.timeout)
        r.raise_for_status()
        return np.asarray(r.json()["actions"], dtype=np.float64)


# ---------------------------------------------------------------- 主回路
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=["local", "server"], default="local")
    ap.add_argument("--ckpt", default="runs/mini_vla/ckpt.pt")
    ap.add_argument("--api", default="http://127.0.0.1:8000")
    ap.add_argument("--task", choices=list(E.TASKS), default="reach")
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--hz", type=float, default=10.0)
    ap.add_argument("--max-steps", type=int, default=E.T)
    args = ap.parse_args()

    robot = DummyRobot(args.task, args.seed)
    safety = SafetyLayer()
    policy = LocalPolicy(args.ckpt) if args.mode == "local" else ServerPolicy(args.api)

    period = 1.0 / args.hz
    consecutive_fail = 0
    print(f"[robot] task={args.task}  instruction={robot.instruction!r}  mode={args.mode}")

    for t in range(args.max_steps):
        t0 = time.perf_counter()
        obs = robot.get_obs()
        try:
            chunk = policy.act(obs)                 # 动作块 (H,2)，异步架构下此处不阻塞
            consecutive_fail = 0
        except Exception as exc:                    # 看门狗：推理故障 → 受控停止
            consecutive_fail += 1
            print(f"[watchdog] inference failed x{consecutive_fail}: {exc}")
            if consecutive_fail >= 3:
                print("[watchdog] give up -> controlled stop")
                return
            time.sleep(period)
            continue

        vel, reason = safety.check(chunk[0], obs["state"])   # 只执行块内第 1 步
        if reason is not None:
            print(f"[safety] drop action: {reason}")
            time.sleep(period)
            continue

        ok = robot.step(vel)
        loop_ms = (time.perf_counter() - t0) * 1000
        print(f"t={t:03d}  vel=({vel[0]:+5.1f},{vel[1]:+5.1f})  loop={loop_ms:5.1f}ms")
        if ok:
            print(f"[success] task '{args.task}' finished in {t+1} steps")
            return
        wait = period - (time.perf_counter() - t0)
        if wait > 0:
            time.sleep(wait)
    print(f"[fail] not finished within {args.max_steps} steps")


if __name__ == "__main__":
    main()
