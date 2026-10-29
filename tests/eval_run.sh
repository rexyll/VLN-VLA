#!/bin/bash
# tests/eval_run.sh <时长s> <输出名> —— 后台跑一轮随机跟随评测
#
# 位置: tests/   依赖: tests/set_follow.py, tests/random_eval.py
# 产物: tests/out/<输出名>.{json,log,done}
set +u
export VLA_HOME="${VLA_HOME:-/home/earl/workspace/VLA}"
export DISPLAY=:0
source /opt/ros/jazzy/setup.bash
source "$VLA_HOME/vla_ws/install/setup.bash"

DUR=${1:-150}
OUT=${2:-eval}
OUTDIR="$VLA_HOME/tests/out"
mkdir -p "$OUTDIR"

rm -f "$OUTDIR/${OUT}.done"
# 固定跟随距离, 免受演示语音"两米"影响
python3 "$VLA_HOME/tests/set_follow.py" 1.0 $((DUR+25)) > /dev/null 2>&1 &
SETPID=$!
python3 "$VLA_HOME/tests/random_eval.py" --dur "$DUR" --out "$OUT" \
    --outdir "$OUTDIR" > "$OUTDIR/${OUT}.log" 2>&1
kill $SETPID 2>/dev/null
touch "$OUTDIR/${OUT}.done"
