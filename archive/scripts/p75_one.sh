#!/bin/bash
# p75_one.sh <tag> <dur> <rec|rpl> —— 单轮 PW_RECORD / PW_REPLAY 实验
# 门禁修正: gz 的进程名是 ruby(gz 是 ruby 包装脚本), pgrep -x gz 恒为 0 是假门禁。
#           pgrep -f "[g]z sim" 的方括号写法既命中真进程, 又不匹配自己的命令行。
set +u
cd /home/earl/workspace/VLA || exit 1
TAG=${1:-a}
DUR=${2:-100}
MODE=${3:-rec}
WALK=walk_rec.jsonl
PWFILE=$([ "$MODE" = "rec" ] && echo "$WALK" || echo "$WALK")

echo ">>> [$TAG/$MODE] 清场"
pkill -9 -f 'ros2 launch vla_follow' 2>/dev/null
pkill -9 -f 'vla_follow/lib' 2>/dev/null
pkill -9 -f parameter_bridge 2>/dev/null
pkill -9 -f '/opt/ros/jazzy/lib/nav2_' 2>/dev/null
pkill -9 -f 'gz sim' 2>/dev/null
pkill -9 -f tmp_p74_det 2>/dev/null
pkill -9 -f tmp_set_follow 2>/dev/null
sleep 4

for i in $(seq 1 40); do
    g=$(pgrep -f "[g]z sim" | wc -l)
    b=$(pgrep -x parameter_bridge | wc -l)
    if [ "$g" -eq 0 ] && [ "$b" -eq 0 ]; then
        echo ">>> [$TAG] 清场完成 (${i}s)"
        break
    fi
    sleep 1
done
g=$(pgrep -f "[g]z sim" | wc -l)
if [ "$g" -ne 0 ]; then
    echo ">>> [$TAG] 清场失败, 仍残留 gz=$g —— 中止, 否则数据脏"
    exit 3
fi
echo ">>> [$TAG] 残留 gz=0 bridge=$(pgrep -x parameter_bridge | wc -l)"

if [ "$MODE" = "rpl" ] && [ ! -s "$WALK" ]; then
    echo ">>> [$TAG] 回放文件 $WALK 不存在或为空 —— 中止"
    exit 4
fi
if [ "$MODE" = "rec" ]; then
    rm -f "$WALK"
fi

rm -f "det_$TAG.json" "det_$TAG.log"
echo ">>> [$TAG] 起录制器"
setsid nohup bash p74_rec.sh "$DUR" "det_$TAG.json" > "det_$TAG.log" 2>&1 < /dev/null &

sleep 3
echo ">>> [$TAG] 起 demo (mode=$MODE)"
if [ "$MODE" = "rec" ]; then
    PWENV="PW_RECORD=/home/earl/workspace/VLA/$WALK PW_SEED=7"
else
    PWENV="PW_REPLAY=/home/earl/workspace/VLA/$WALK"
fi
env DISPLAY=:0 $PWENV nohup bash vla_demo/start_demo.sh > "demo_$TAG.log" 2>&1 < /dev/null &

sleep 32
setsid nohup python3 tmp_set_follow.py 1.0 $((DUR + 40)) > /dev/null 2>&1 < /dev/null &

for i in $(seq 1 70); do
    sleep 5
    if grep -q 'wrote' "det_$TAG.log" 2>/dev/null; then
        echo ">>> [$TAG] 录制完成 ($(($i * 5))s 轮询)"
        break
    fi
done
pkill -f tmp_set_follow 2>/dev/null

echo ">>> [$TAG] PERSON 模式确认:"
grep -m2 -E 'PERSON (录制|回放)模式|PERSON\(回放\)' "demo_$TAG.log" "vla_ws/logs/demo.log" 2>/dev/null | head -4
echo ">>> [$TAG] det_$TAG.log:"
cat "det_$TAG.log"
if [ "$MODE" = "rec" ]; then
    echo ">>> [$TAG] $WALK: $(wc -l < "$WALK" 2>/dev/null) 行"
fi
