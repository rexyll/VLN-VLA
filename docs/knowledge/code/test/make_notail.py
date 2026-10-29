# -*- coding: utf-8 -*-
"""make_notail.py —— 去掉成功后的零动作尾帧（tlen = success_t + 1）

动机：零动作尾帧教会模型"靠近目标→停车"，且该行为会外推到 8~14px 造成
闭环停滞（docs/09 §12 #7）。去掉尾帧后，"持续逼近目标"成为唯一语义；
到达成功区即由评估/部署侧终止回合。沿用 imgs 等已有数组，仅改 tlen。
用法: python3 make_notail.py <src.npz> <dst.npz>
"""
import sys

import numpy as np

src, dst = sys.argv[1], sys.argv[2]
d = dict(np.load(src, allow_pickle=True))
d["tlen"] = (d["success_t"] + 1).astype(np.int32)
np.savez_compressed(dst, **d)
print(f"saved {dst}  episodes={len(d['tlen'])}  frames={int(d['tlen'].sum())}")
