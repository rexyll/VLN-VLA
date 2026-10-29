#!/usr/bin/env bash
# vm_rollout_trace.sh —— 闭环 trace：模型动作幅度 + 辅助头定位 + 距离曲线
cd /home/earl/vla/code/test
python3 - <<'EOF'
import sys, time
sys.path.insert(0, ".")
sys.path.insert(0, "../mini_vla")
import numpy as np, torch
from gz_env import GzArenaEnv
from evaluate import load_model
from dataset import encode_text
import env_synth as E

model, cfg, stats, vocab, meta = load_model("runs/gz_model/ckpt.pt", "runs/gz_model")
model.to("cpu")
env = GzArenaEnv()

for seed in (5000, 5001, 5002):
    env.reset("reach", seed)
    st = env.state_px()
    ag, tgt = st["agent"].copy(), st["red"].copy()
    lang, mask = encode_text(env.text, vocab)
    lang_t = torch.from_numpy(lang)[None]
    mask_t = torch.from_numpy(mask)[None]
    print(f"--- seed {seed} 初始距离={np.linalg.norm(ag-tgt):.1f}px  text={env.text!r}")
    for t in range(24):
        img = env.image()
        state = (ag / (E.S / 2) - 1).astype(np.float32)
        x = torch.from_numpy(img).permute(2, 0, 1)[None].float() / 255.0
        with torch.no_grad():
            chunk, aux = model(x, lang_t, mask_t,
                               torch.from_numpy(state)[None], return_aux=True)
        aux_px = (aux[0].numpy() + 1) * (E.S / 2)
        aux_err = float(np.linalg.norm(aux_px - tgt))
        a = chunk[0, 0].numpy()
        mag = float(np.linalg.norm(a))
        print(f"  t={t:02d} dist={np.linalg.norm(ag-tgt):5.1f} |a|={mag:5.1f} "
              f"a={a.round(1).tolist()} aux_err={aux_err:5.1f}", flush=True)
        ag, _ = E.env_step(ag, st["box"], "reach",
                           np.clip(a, -E.MAX_SPEED, E.MAX_SPEED))
        env.vel(float(np.clip(a[0], -E.MAX_SPEED, E.MAX_SPEED)),
                float(np.clip(a[1], -E.MAX_SPEED, E.MAX_SPEED)))
        time.sleep(E.DT)
    env.stop()
EOF
