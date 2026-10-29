#!/usr/bin/env bash
# vm_input_compare.sh —— 决定性对照：训练样本帧 vs 实时帧 + 模型在两者上的输出
cd /home/earl/vla/code/test
python3 - <<'EOF'
import sys
sys.path.insert(0, ".")
sys.path.insert(0, "../mini_vla")
import numpy as np, torch
from gz_env import GzArenaEnv
from evaluate import load_model
from dataset import normalize, unnormalize, encode_text
import env_synth as E

d = np.load("gz_data_big.npz", allow_pickle=True)
seeds = d["seed"]
e = int(np.where(seeds == 5000)[0][0])
task = str(d["task"][e])
print(f"episode e={e} task={task} tlen={int(d['tlen'][e])} text={str(d['text'][e])!r}")
print("stored actions[0:5]=", d["actions"][e, :5].round(1).tolist())
print("action stats: min", d["actions"].min().round(1), "max", d["actions"].max().round(1),
      "mean|a|", float(np.abs(d["actions"]).mean()).__round__(1))

model, cfg, stats, vocab, meta = load_model("runs/gz_model/ckpt.pt", "runs/gz_model")

def run(img, state, text):
    lang, mask = encode_text(text, vocab)
    x = torch.from_numpy(np.ascontiguousarray(img)).permute(2, 0, 1)[None].float() / 255.0
    s = torch.from_numpy(np.asarray(state, np.float32))[None]
    with torch.no_grad():
        chunk, aux = model(x, torch.from_numpy(lang)[None],
                           torch.from_numpy(mask)[None], s, return_aux=True)
    return unnormalize(chunk[0, 0].numpy(), stats)

stored_img = d["images"][e, 0]
stored_state = d["agent"][e, 0]
stored_text = str(d["text"][e])
out_stored = run(stored_img, stored_state, stored_text)
tgt_stored = unnormalize(d["actions"][e, 0], stats)
print(f"模型@训练样本: chunk0={out_stored.round(1).tolist()}  数据目标={tgt_stored.round(1).tolist()}")

env = GzArenaEnv()
env.reset(task, 5000)
live = env.image()
st = env.state_px()
out_live = run(live, st["agent"], env.text)
print(f"模型@实时帧  : chunk0={out_live.round(1).tolist()}  (真值状态 agent={st['agent'].round(1).tolist()})")
print("帧差异: max", int(np.abs(live.astype(int) - stored_img.astype(int)).max()),
      " mean", float(np.abs(live.astype(int) - stored_img.astype(int)).mean()).__round__(2))
print("状态差异:", (st["agent"] - stored_state).round(2).tolist())
EOF
