#!/usr/bin/env bash
# run_demos.sh —— 顺序运行三个 demo 并保存输出
# 用法: bash run_demos.sh [ckpt路径]
cd /home/earl/vla/code/test
mkdir -p results
CKPT="${1:-runs/gz_model/ckpt.pt}"
OUT=results/demos_output.txt
echo "ckpt=$CKPT" | tee "$OUT"
for d in 1 2 3; do
  echo "===== DEMO $d =====" | tee -a "$OUT"
  python3 "demo${d}_"*.py --ckpt "$CKPT" 2>&1 | tee -a "$OUT"
done
echo "ALL-DEMOS-DONE" | tee -a "$OUT"
