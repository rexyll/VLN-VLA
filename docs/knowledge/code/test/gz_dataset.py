# -*- coding: utf-8 -*-
"""gz_dataset.py —— 使用 Gazebo 实拍帧（而非合成渲染）的数据集子类

继承 MiniVLADataset，仅覆写 _frame：图像来自 npz 的 images 字段
（Gazebo 顶视相机实拍 + 与部署一致的域差校正），其余逻辑（动作块、
归一化、词表、tlen 截断）与合成数据完全一致。
"""
from __future__ import annotations

import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / "mini_vla"))
from dataset import MiniVLADataset  # noqa: E402


class GzVLADataset(MiniVLADataset):
    def _frame(self, e: int, t: int, seed: int, bg: int, task: str) -> np.ndarray:
        return self._d["images"][e, t]
