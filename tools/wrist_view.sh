#!/bin/bash
# wrist_view.sh —— 起/停腕部 RGB-D 实时预览窗口(给人在屏幕上看)
#
# 为什么单独一个脚本: 在 ssh 一条命令里写 `... &` 会和后面的检查命令抢进程组,
# 远端 shell 退出时把窗口一起带走(实测: 命令超时被杀, 窗口根本没留下)。
# 放进脚本 + setsid, 进程脱离当前会话, ssh 断开也不影响。
#
# 用法:
#   bash tools/wrist_view.sh            # 起(RGB+depth, 1.2 倍)
#   bash tools/wrist_view.sh 1.6        # 起, 指定缩放
#   bash tools/wrist_view.sh stop       # 关
#
# 窗口内容: 左列右手 / 右列左手, 上排 RGB 下排深度(近=黑 远=白, 0.05~1.0m)。
# 按键: q/ESC 退出, d 去掉深度行, s 存图, +/- 缩放。
# 话题: gz 原生 /wrist_cam_R|L/{image,depth_image} —— 不经过 ROS 桥, 少一层延迟。
export DISPLAY="${DISPLAY:-:0}"
export VLA_HOME="${VLA_HOME:-/home/earl/workspace/VLA}"
LOG="$VLA_HOME/vla_ws/logs/wristview.log"

if [ "${1:-}" = "stop" ]; then
    pkill -f 'tools/wrist_view.py' 2>/dev/null
    sleep 1
    echo ">>> 腕部预览窗口已关闭"
    exit 0
fi

SCALE="${1:-1.2}"
pkill -f 'tools/wrist_view.py' 2>/dev/null
sleep 1
cd "$VLA_HOME" || exit 1
setsid nohup env WRIST_VIEW_SCALE="$SCALE" python3 tools/wrist_view.py \
    > "$LOG" 2>&1 < /dev/null &
sleep 6

n=$(ps -eo args --no-headers | grep -c '[w]rist_view.py')
echo ">>> 预览进程数: $n  缩放: $SCALE"
if [ "$n" = "0" ]; then
    tail -8 "$LOG"
    exit 1
fi

# 摆位交给 gui_arrange.sh: mutter 会覆盖只设一次的 wmctrl 几何, 那边有回读重试。
# 尺寸 = 画布尺寸(2 列 x 2 行, 每格 256x192 放大 SCALE 倍): cv2 的 WINDOW_NORMAL
# 允许拖拽缩放, 但内容不会跟着缩, 窗口小了就被裁掉 —— 按内容尺寸定死。
W=$(python3 -c "print(int(2 * 256 * $SCALE))")
H=$(python3 -c "print(int(2 * 192 * $SCALE))")
for i in 1 2 3 4; do
    wmctrl -l 2>/dev/null | grep -q 'cams' && break
    sleep 1
done
bash "$VLA_HOME/tools/gui_arrange.sh" wrist "$W" "$H"
echo ">>> 尺寸 ${W}x${H}  按键: q 退出  d 去深度  s 存图  +/- 缩放"
