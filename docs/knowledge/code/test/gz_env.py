# -*- coding: utf-8 -*-
"""gz_env.py —— Gazebo 仿真环境客户端：与 mini_vla.DummyRobot 相同的机器人接口

桥接: test/gazebo/libvla_bridge.so（TCP 行协议），单位换算 1px = 1cm。
标定: 首次运行 calibrate.py 生成 calib.json（图像朝向自检）；
      默认假设图像 col=x、row=y，与合成渲染一致。
域差校正: 背景灰度自动增益到合成训练分布的中位 26（docs/12 §16 的最小实现）。
"""
from __future__ import annotations

import json
import pathlib
import socket
import sys
import time

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "mini_vla"))
import env_synth as E  # noqa: E402

PX_PER_M = 100.0
DT = E.DT


class BridgeError(RuntimeError):
    pass


class GzArenaEnv:
    """接口与 mini_vla.DummyRobot 对齐: get_obs / step / reset，另有 stop()/state_px()。"""

    def __init__(self, host: str = "127.0.0.1", port: int = 9101,
                 auto_gain: bool = True, wait: float = 90.0):
        self.auto_gain = auto_gain
        self.calib = {"swap": False, "flip_col": False, "flip_row": False}
        p = pathlib.Path(__file__).parent / "calib.json"
        if p.exists():
            self.calib.update(json.loads(p.read_text("utf-8")))
        self.task = "reach"
        self.target = np.zeros(2)
        self.text = ""
        self._buf = b""
        self._connect(host, port, wait)

    # ---------------- 底层连接 ----------------
    def _connect(self, host: str, port: int, wait: float) -> None:
        t0 = time.time()
        while True:
            try:
                self.sock = socket.create_connection((host, port), timeout=5)
                break
            except OSError:
                if time.time() - t0 > wait:
                    raise BridgeError("无法连接 vla_bridge —— 请先运行 bash run_gazebo.sh")
                time.sleep(1.0)
        deadline = time.time() + 30
        while time.time() < deadline:
            try:
                if self._cmd("PING").startswith("PONG"):
                    return
            except (OSError, BridgeError):
                pass
            time.sleep(0.5)
        raise BridgeError("bridge 无 PONG 响应")

    def _recv_line(self) -> str:
        while b"\n" not in self._buf:
            d = self.sock.recv(262144)
            if not d:
                raise BridgeError("bridge 连接断开")
            self._buf += d
        line, self._buf = self._buf.split(b"\n", 1)
        return line.decode().strip()

    def _cmd(self, line: str) -> str:
        self.sock.sendall((line + "\n").encode())
        return self._recv_line()

    # ---------------- 状态与图像 ----------------
    def state_px(self) -> dict:
        parts = self._cmd("STATE").split()
        if parts[0] != "STATE":
            raise BridgeError(f"STATE 响应异常: {parts}")
        m = [float(v) for v in parts[2:10]]
        px = [v * PX_PER_M for v in m]
        return {"t": float(parts[1]),
                "agent": np.array(px[0:2]), "box": np.array(px[2:4]),
                "red": np.array(px[4:6]), "yellow": np.array(px[6:8]),
                "contact": int(parts[10])}

    def raw_image(self) -> np.ndarray:
        hdr = self._cmd("GET_IMAGE")
        if hdr.startswith("ERR"):
            raise BridgeError("GET_IMAGE: " + hdr)
        _, w, h, n = hdr.split()
        w, h, n = int(w), int(h), int(n)
        while len(self._buf) < n:
            d = self.sock.recv(262144)
            if not d:
                raise BridgeError("bridge 连接断开")
            self._buf += d
        data, self._buf = self._buf[:n], self._buf[n:]
        return np.frombuffer(data, np.uint8).reshape(h, w, 3).copy()

    def image(self) -> np.ndarray:
        """策略观测帧：按标定变换朝向 + 背景自动增益（域差校正）。"""
        img = self.raw_image()
        if self.calib.get("swap"):
            img = img.transpose(1, 0, 2)
        if self.calib.get("flip_col"):
            img = img[:, ::-1]
        if self.calib.get("flip_row"):
            img = img[::-1]
        if self.auto_gain:
            med = float(np.median(img))
            if med > 5:
                img = np.clip(img.astype(np.float32) * (26.0 / med), 0, 255).astype(np.uint8)
        return np.ascontiguousarray(img)

    def img_status(self) -> str:
        return self._cmd("IMG_STATUS")

    # ---------------- 控制 ----------------
    def vel(self, vx: float, vy: float) -> None:
        """速度指令，单位 px/s（内部换算为 m/s 下发）。"""
        self._cmd(f"VEL {vx * 0.01:.4f} {vy * 0.01:.4f}")

    def stop(self) -> None:
        self._cmd("STOP")

    def set_pose(self, who: str, px: float, py: float) -> None:
        """传送模型（who: agent|box|red|yellow），单位 px。"""
        self._cmd(f"SET_POSE {who} {px / PX_PER_M:.4f} {py / PX_PER_M:.4f}")

    # ---------------- 与 mini_vla 相同的机器人接口 ----------------
    def _sample_start(self, task: str, rng: np.random.Generator):
        for _ in range(100):
            target = rng.uniform(10, E.S - 11, 2)
            agent = rng.uniform(6, E.S - 7, 2)
            box = rng.uniform(8, E.S - 9, 2)
            if (np.linalg.norm(agent - target) > 14
                    and np.linalg.norm(box - target) > (14 if task == "push" else 6)
                    and np.linalg.norm(agent - box) > 6):
                return target, agent, box
        raise RuntimeError("采样失败")

    def reset(self, task: str, seed: int) -> dict:
        """随机化初始位姿（与训练同分布）并等待物理沉降。"""
        rng = np.random.default_rng(seed)
        target, agent, box = self._sample_start(task, rng)
        red = target if task == "reach" else np.array([56.0, 8.0])
        yel = target if task == "push" else np.array([56.0, 8.0])
        self._cmd("RESET "
                  f"{agent[0] / PX_PER_M:.4f} {agent[1] / PX_PER_M:.4f} "
                  f"{box[0] / PX_PER_M:.4f} {box[1] / PX_PER_M:.4f} "
                  f"{red[0] / PX_PER_M:.4f} {red[1] / PX_PER_M:.4f} "
                  f"{yel[0] / PX_PER_M:.4f} {yel[1] / PX_PER_M:.4f}")
        time.sleep(0.3)                       # 物理沉降
        for _ in range(10):                   # 等待箱子静止（清除残余速度）
            a, b = self.state_px()["box"], self.state_px()["box"]
            if np.linalg.norm(a - b) < 0.1:
                break
            time.sleep(0.1)
        self.task = task
        self.target = np.asarray(target, dtype=np.float64)
        self.text = E.TEXTS[task][seed % len(E.TEXTS[task])]
        st = self.state_px()
        return {"agent": st["agent"], "box": st["box"]}

    def get_obs(self) -> dict:
        st = self.state_px()
        return {"image": self.image(), "state": st["agent"],
                "instruction": self.text, "box": st["box"],
                "contact": st["contact"]}

    def step(self, vel) -> bool:
        """下发一个控制周期的速度指令并返回任务是否成功。"""
        self.vel(float(vel[0]), float(vel[1]))
        time.sleep(DT)
        st = self.state_px()
        return bool(E.is_success(st["agent"], st["box"], self.target, self.task))
