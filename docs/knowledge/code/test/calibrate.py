# -*- coding: utf-8 -*-
"""calibrate.py —— 虚拟相机像素↔世界映射自检与朝向标定

原理：把蓝色智能体传送到 4 个已知像素位置，从虚拟相机取帧并检测蓝色色块质心，
比较 (col,row) 与 (x_px,y_px) 的关系 → 自动判定是否需要转置/翻转 → 写 calib.json。

运行（gzserver 已启动）: python3 calibrate.py
"""
from __future__ import annotations

import json
import pathlib
import sys
import time

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from gz_env import GzArenaEnv  # noqa: E402

PROBES = [(16, 16), (16, 48), (48, 16), (48, 48)]


def blue_centroid(img: np.ndarray):
    b = img[:, :, 2].astype(int)
    r = img[:, :, 0].astype(int)
    g = img[:, :, 1].astype(int)
    mask = (b - r > 40) & (b - g > 40) & (b > 90)
    ys, xs = np.where(mask)
    if len(xs) < 3:
        return None
    return float(xs.mean()), float(ys.mean())  # (col, row)


def main():
    env = GzArenaEnv()
    print("IMG_STATUS:", env.img_status())
    obs = []  # (x_px, y_px, col, row)
    for px, py in PROBES:
        env.set_pose("agent", px, py)
        time.sleep(0.4)
        img = env.raw_image()  # 用原始帧，排除 calib 变换干扰
        c = blue_centroid(img)
        if c is None:
            print(f"probe ({px},{py}): 蓝色色块未检出！检查材质/光照/相机")
            continue
        obs.append((px, py, c[0], c[1]))
        print(f"probe world=({px:4.1f},{py:4.1f})px -> image (col={c[0]:5.1f}, row={c[1]:5.1f})")
    if len(obs) < 3:
        print("CALIB-FAIL: 有效探测点不足")
        return

    # 候选映射：col/row 与 x/y 的 8 种二面体关系，选平均误差最小者
    def err(swap, fc, fr):
        es = []
        for x, y, col, row in obs:
            c, r = (y, x) if swap else (x, y)
            c = (64 - c) if fc else c
            r = (64 - r) if fr else r
            es.append(np.hypot(col - c, row - r))
        return float(np.mean(es))

    candidates = [((s, fc, fr), err(s, fc, fr))
                  for s in (0, 1) for fc in (0, 1) for fr in (0, 1)]
    (swap, fc, fr), e = min(candidates, key=lambda kv: kv[1])
    print(f"最优映射: swap={bool(swap)} flip_col={bool(fc)} flip_row={bool(fr)}  "
          f"平均误差={e:.2f}px")
    out = {"swap": bool(swap), "flip_col": bool(fc), "flip_row": bool(fr), "err_px": e}
    pathlib.Path(__file__).parent.joinpath("calib.json").write_text(
        json.dumps(out, indent=1), "utf-8")
    print("CALIB-OK -> calib.json" if e < 2.5 else f"CALIB-WARN: 误差偏大 {e:.2f}px")


if __name__ == "__main__":
    main()
