#!/usr/bin/env bash
# vm_frame_debug.sh —— 拉取 demo 帧并打印统计与色块检测
cd /home/earl/vla/code/test
python3 - <<'EOF'
import sys
sys.path.insert(0, ".")
sys.path.insert(0, "../mini_vla")
import numpy as np
from gz_env import GzArenaEnv

env = GzArenaEnv()
env.reset("reach", 1)
raw = env.raw_image()
img = env.image()
print("raw  : shape", raw.shape, "median", float(np.median(raw)),
      "mean", raw.mean(axis=(0,1)).round(1).tolist())
print("gained: median", float(np.median(img)), "mean", img.mean(axis=(0,1)).round(1).tolist())
for name, cond in [
    ("blue", (img[:,:,2].astype(int)-img[:,:,0] > 40) & (img[:,:,2].astype(int)-img[:,:,1] > 40) & (img[:,:,2] > 90)),
    ("red",  (img[:,:,0] > 120) & (img[:,:,1] < 100) & (img[:,:,2] < 100)),
    ("green",(img[:,:,1].astype(int)-img[:,:,0] > 40) & (img[:,:,1] > 90)),
]:
    ys, xs = np.where(cond)
    c = (float(xs.mean()), float(ys.mean())) if len(xs) else None
    print(f"blob {name}: n={len(xs)} centroid={c}")
st = env.state_px()
print("truth: agent=", st["agent"].round(1).tolist(), "red=", st["red"].round(1).tolist())
from PIL import Image
Image.fromarray(img).save("results/debug_frame.png")
Image.fromarray(raw).save("results/debug_raw.png")
print("saved results/debug_frame.png & debug_raw.png")
EOF
