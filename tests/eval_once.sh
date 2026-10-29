#!/bin/bash
# tests/eval_once.sh <dur> <out> [rec|rpl] —— 干净重启 + 标准口径评测(可开回放)
#
# 位置: tests/   依赖: tests/eval_run.sh, tests/data/walk_rec.jsonl
# 门禁修正: pgrep -x gz 恒为 0(真进程名是 ruby), 用 pgrep -f "[g]z sim"。
set +u
export VLA_HOME="${VLA_HOME:-/home/earl/workspace/VLA}"
cd "$VLA_HOME" || exit 1

DUR=${1:-120}
OUT=${2:-e}
MODE=${3:-rpl}
WALK=tests/data/walk_rec.jsonl
OUTDIR="$VLA_HOME/tests/out"
mkdir -p "$OUTDIR"

pkill -9 -f 'ros2 launch vla_follow' 2>/dev/null
pkill -9 -f 'vla_follow/lib' 2>/dev/null
pkill -9 -f parameter_bridge 2>/dev/null
pkill -9 -f '/opt/ros/jazzy/lib/nav2_' 2>/dev/null
pkill -9 -f 'gz sim' 2>/dev/null
pkill -9 -f 'tests/set_follow.py' 2>/dev/null
sleep 4
for i in $(seq 1 40); do
    [ "$(pgrep -f "[g]z sim" | wc -l)" -eq 0 ] && break
    sleep 1
done
if [ "$(pgrep -f "[g]z sim" | wc -l)" -ne 0 ]; then
    echo ">>> 清场失败"; exit 3
fi

if [ "$MODE" = "rpl" ] && [ ! -s "$VLA_HOME/$WALK" ]; then
    echo ">>> 回放文件缺失: $WALK"; exit 4
fi
if [ "$MODE" = "rpl" ]; then
    PWENV="PW_REPLAY=$VLA_HOME/$WALK"
    echo ">>> 起 demo (REPLAY $WALK)"
else
    PWENV="PW_SEED=7"
    echo ">>> 起 demo (SEED=7)"
fi
env DISPLAY=:0 $PWENV nohup bash "$VLA_HOME/vla_demo/start_demo.sh" \
    > "$OUTDIR/demo_$OUT.log" 2>&1 < /dev/null &

for i in $(seq 1 30); do
    sleep 3
    if [ "$MODE" = "rpl" ]; then
        grep -q 'PERSON(回放)' vla_ws/logs/demo.log 2>/dev/null && break
    else
        grep -q 'PERSON 行走' vla_ws/logs/demo.log 2>/dev/null && break
    fi
done
sleep 8
wc -l < vla_ws/logs/demo.log > "/tmp/$OUT.mark"
echo ">>> 评测 $DUR s"
bash "$VLA_HOME/tests/eval_run.sh" "$DUR" "$OUT" 2>&1 | tail -3
echo ">>> 完成: $OUTDIR/$OUT.json"
