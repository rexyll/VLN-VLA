# -*- coding: utf-8 -*-
"""
env_synth.py —— 合成 2D 操作环境 + 脚本专家 + 数据生成器（教学用最小 VLA 闭环）

设计目的
    用一个能在 CPU 上秒级生成的合成任务族，跑通 VLA 的"数据 → 训练 → 部署"全流程，
    重点示范工程结构而非任务难度。真实机器人只需替换本文件的"环境接口"，
    dataset / model / train / server / robot_loop 全部不变 —— 这正是模板强调的接口稳定原则。

任务族（语言指令真正影响行为：同一画面布局，指令决定任务）
    reach : "move the blue square to the red cross"     —— 把蓝色方块(智能体)移到红十字
    push  : "push the green box to the yellow cross"    —— 把绿色方块推到黄十字
    画面中同时存在蓝方块、绿方块、红十字、黄十字，二者互为干扰物。

状态/动作约定（写进元数据，部署端必须一致 —— 见 docs/04-数据体系 §4、docs/09-易错知识点）
    观测: 64x64 RGB uint8 图像 + 智能体位置 (x, y) ∈ [0, 64)（本体感知，像素单位）
    动作: 速度指令 [vx, vy]，单位 px/s，范围 [-40, 40]，控制周期 dt=0.1s
    成功: reach 判智能体与红十字距离 < 4px；push 判绿箱与黄十字距离 < 4px
"""
from __future__ import annotations

import numpy as np
from dataclasses import dataclass, asdict

S = 64                 # 图像边长
DT = 0.1               # 控制周期（秒）
MAX_SPEED = 40.0       # 动作上限 px/s
T = 60                 # 每 episode 最大步数
SUCCESS_RADIUS = 4.0   # 判定半径
AGENT_HALF = 2         # 蓝色智能体 5x5
BOX_HALF = 4           # 绿色箱子 9x9
CONTACT_DIST = 7.0     # 智能体与箱子的接触判定距离（push 生效条件）
APPROACH_DIST = 8.0    # 推箱前置点：绕到箱子背对目标一侧
KP = 8.0               # 专家 P 控制增益
EXPERT_NOISE = 2.0     # 专家速度噪声（让演示带一点自然抖动）
STOP_TAIL = 6          # 成功后录制的“停止”帧数（避免数据被大量零动作帧主导）

BLUE = (40, 90, 220)
GREEN = (40, 170, 70)
RED = (215, 45, 45)
YELLOW = (230, 200, 40)

TASKS = ("reach", "push")
TEXTS = {
    "reach": ["move the blue square to the red cross",
              "bring the blue block onto the red cross",
              "put the blue square at the red target"],
    "push":  ["push the green box to the yellow cross",
              "slide the green block onto the yellow target",
              "move the green box to the yellow cross"],
}


def _draw_cross(img: np.ndarray, xy, color, r: int = 3) -> None:
    x, y = int(round(xy[0])), int(round(xy[1]))
    y0, y1 = max(0, y - r), min(S - 1, y + r)
    x0, x1 = max(0, x - r), min(S - 1, x + r)
    img[y0:y1 + 1, x] = color     # 竖臂
    img[y, x0:x1 + 1] = color     # 横臂


def _draw_square(img: np.ndarray, xy, half: int, color) -> None:
    x, y = int(round(xy[0])), int(round(xy[1]))
    y0, y1 = max(0, y - half), min(S - 1, y + half)
    x0, x1 = max(0, x - half), min(S - 1, x + half)
    img[y0:y1 + 1, x0:x1 + 1] = color


def render_frame(agent, box, target, task: str, bg: int, noise_seed: int) -> np.ndarray:
    """确定性渲染：同一 (状态, 种子) 永远得到同一帧 —— 可复现数据管线的基础。"""
    rng = np.random.default_rng(noise_seed)
    img = np.full((S, S, 3), bg, dtype=np.uint8)
    _draw_cross(img, target, RED if task == "reach" else YELLOW)
    _draw_square(img, box, BOX_HALF, GREEN)      # 两个任务都画箱子：语言才不可省略
    _draw_square(img, agent, AGENT_HALF, BLUE)
    noise = rng.normal(0.0, 2.0, img.shape)      # 感知噪声：逼真且防过拟合
    return np.clip(img.astype(np.float32) + noise, 0, 255).astype(np.uint8)


def env_step(agent, box, task: str, vel, dt: float = DT):
    """一步动力学。vel: px/s。push 任务中，接触时箱子随智能体位移。"""
    vel = np.clip(np.asarray(vel, dtype=np.float64), -MAX_SPEED, MAX_SPEED)
    delta = vel * dt
    agent_new = np.clip(agent + delta, AGENT_HALF, S - 1 - AGENT_HALF)
    box_new = box
    if task == "push" and np.linalg.norm(agent_new - box) < CONTACT_DIST:
        box_new = np.clip(box + delta, BOX_HALF, S - 1 - BOX_HALF)
    return agent_new, box_new


def is_success(agent, box, target, task: str) -> bool:
    p = agent if task == "reach" else box
    return bool(np.linalg.norm(np.asarray(p) - np.asarray(target)) < SUCCESS_RADIUS)


def expert_action(agent, box, target, task: str, rng: np.random.Generator) -> np.ndarray:
    """脚本专家：reach 直奔目标；push 先绕到箱子背后再穿箱推行。

    push 带绕行逻辑：若智能体位于“目标一侧”（直接推进会推离目标），
    先径向脱离接触，再从侧向绕到接近点 —— 否则错误侧出生的场景必败。
    """
    if task == "reach":
        goal = np.asarray(target, dtype=np.float64)
    else:
        d = np.asarray(target) - box
        nd = np.linalg.norm(d) + 1e-8
        u = d / nd                                    # 箱子→目标 单位向量
        approach = box - u * APPROACH_DIST            # 接近点：背对目标一侧
        rel = agent - box
        side = float(np.dot(rel, u))                  # >0：智能体在目标一侧
        in_contact = np.linalg.norm(rel) < CONTACT_DIST + 1.0
        if in_contact and side > 0:
            goal = agent + rel / (np.linalg.norm(rel) + 1e-8) * 6.0   # 径向脱离
        elif side > 0 and np.linalg.norm(approach - agent) > 3.0:
            perp = np.array([-u[1], u[0]]) * (1.0 if rng.uniform() < 0.5 else -1.0)
            goal = approach + perp * 8.0                              # 侧向绕行
        elif np.linalg.norm(approach - agent) < 3.0:
            goal = box + u * 2.0                                      # 穿箱推行
        else:
            goal = approach
    v = (goal - agent) * KP + rng.normal(0, EXPERT_NOISE, 2)
    # 先加噪声再裁剪：记录的动作必须与实际执行的一致（docs/09：记录≠执行是典型数据事故）
    return np.clip(v, -MAX_SPEED, MAX_SPEED)


@dataclass
class Episode:
    agent: np.ndarray   # (T,2) 智能体轨迹
    box: np.ndarray     # (T,2) 箱子轨迹
    target: np.ndarray  # (2,)
    actions: np.ndarray # (T,2) 专家速度指令 px/s
    task: str
    text: str
    bg: int
    seed: int
    success_t: int      # 成功发生的时间步；T 表示失败（生成时被丢弃）
    tlen: int           # 有效帧数 = 成功步 + 停止尾帧（截断零动作长尾）


def gen_episode(seed: int, task: str, max_tries: int = 50) -> Episode | None:
    """生成一条成功演示；初值不满足约束或专家失败则换种子重试。"""
    rng = np.random.default_rng(seed)
    for _ in range(max_tries):
        target = rng.uniform(10, S - 11, 2)
        agent = rng.uniform(6, S - 7, 2)
        box = rng.uniform(8, S - 9, 2)
        ok = (np.linalg.norm(agent - target) > 14 and
              np.linalg.norm(box - target) > (14 if task == "push" else 6) and
              np.linalg.norm(agent - box) > 6)
        if ok:
            break
    else:
        return None

    # 时序约定（docs/09 头号坑之一：观测-动作因果对齐）
    # state[t] = 执行动作前的观测；actions[t] = 从该观测出发的动作指令
    a_traj = np.zeros((T, 2))
    ag_traj = np.zeros((T, 2)); bx_traj = np.zeros((T, 2)); done_t = T
    ag, bx = agent.copy(), box.copy()
    for t in range(T):
        v = expert_action(ag, bx, target, task, rng) if done_t == T else np.zeros(2)
        ag_traj[t] = ag; bx_traj[t] = bx; a_traj[t] = v    # 先记录“观测-动作”对
        ag, bx = env_step(ag, bx, task, v)                 # 再推进一步
        if done_t == T and is_success(ag, bx, target, task):
            done_t = t
    if done_t == T:
        return None                                    # 专家失败：丢弃（BC 数据只要成功演示）
    tlen = int(min(T, done_t + 1 + STOP_TAIL))         # 成功后只保留少量“停止”演示帧
    text = TEXTS[task][seed % len(TEXTS[task])]
    return Episode(ag_traj, bx_traj, target, a_traj, task, text,
                   int(rng.integers(18, 36)), seed, done_t, tlen)


def gen_dataset(n_per_task: int = 200, base_seed: int = 0):
    """批量生成成功演示，返回 list[Episode]（仅标量，图像按需确定性渲染）。"""
    eps = []
    for i, task in enumerate(TASKS):
        got, seed = 0, base_seed * 1000 + i * 100000
        while got < n_per_task:
            ep = gen_episode(seed, task)
            seed += 1
            if ep is not None:
                eps.append(ep); got += 1
    return eps


def episodes_to_arrays(eps: list[Episode]) -> dict:
    arrs = {
        "agent": np.stack([e.agent for e in eps]).astype(np.float32),
        "box": np.stack([e.box for e in eps]).astype(np.float32),
        "target": np.stack([e.target for e in eps]).astype(np.float32),
        "actions": np.stack([e.actions for e in eps]).astype(np.float32),
        "success_t": np.array([e.success_t for e in eps], dtype=np.int32),
        "tlen": np.array([e.tlen for e in eps], dtype=np.int32),
        "bg": np.array([e.bg for e in eps], dtype=np.int32),
        "seed": np.array([e.seed for e in eps], dtype=np.int64),
        "task": np.array([e.task for e in eps]),
        "text": np.array([e.text for e in eps]),
    }
    return arrs


if __name__ == "__main__":
    import pathlib, sys
    out = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else "data_synth.npz")
    eps = gen_dataset(n_per_task=int(sys.argv[2]) if len(sys.argv) > 2 else 200)
    np.savez_compressed(out, **episodes_to_arrays(eps))
    print(f"saved {len(eps)} episodes -> {out}")
