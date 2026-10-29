#!/bin/bash
# gui_arrange.sh —— 把桌面上的两个窗口摆到位(Gazebo 3D 铺满 + 腕部预览钉在右下角)
#
# 为什么要单独一个脚本、还要反复设:
#   mutter 对**刚映射**的窗口会重新摆位 —— 在 open_gui.sh / wrist_view.sh 里
#   `wmctrl -r ... -e` 只设一次, 实测过几秒就被 WM 摆回默认(3D 视图缩成
#   910x539 挤在中间、腕部窗口跑到左上角), 看上去像"脚本没生效"。
#   这里改成: 按窗口 id 操作 + **回读几何校验**, 不合格就再来一次。
#
# 另一个坑: `xdotool search --name` 会同时命中**外框窗口**(Qt/WM 加的那层)和
# 真正的**客户端窗口**。拿 head -1 去 resize 就改在了外框上 —— 命令全返回 0、
# 窗口纹丝不动, 看起来像"xdotool 不生效"。所以 id 一律从 `wmctrl -l` 取
# (它只列客户端窗口, 且 wmctrl -e 接受它给的十六进制 id)。
#
# 用法:
#   bash tools/gui_arrange.sh                # 两个窗口都摆
#   bash tools/gui_arrange.sh gazebo         # 只铺 Gazebo
#   bash tools/gui_arrange.sh wrist 614 460  # 只摆腕部预览, 指定画布尺寸
export DISPLAY="${DISPLAY:-:0}"
MODE="${1:-all}"
CW="${2:-}"
CH="${3:-}"

SCREEN=$(xdpyinfo 2>/dev/null | awk '/dimensions:/{print $2; exit}')
SW=${SCREEN%x*}; SH=${SCREEN#*x}
[ -z "$SW" ] && SW=1280
[ -z "$SH" ] && SH=800
TITLE_H=32          # 标题栏高度: 窗口高 = 画布高 + 这条

GZ=$(wmctrl -l 2>/dev/null | awk '/Gazebo Sim/{print $1; exit}')
WS=$(wmctrl -l 2>/dev/null | awk '/cams/{print $1; exit}')

if [ "$MODE" != "wrist" ] && [ -n "$GZ" ]; then
    GZD=$((GZ))
    for i in 1 2 3; do
        xdotool windowstate --add MAXIMIZED_VERT "$GZD" 2>/dev/null
        xdotool windowstate --add MAXIMIZED_HORZ "$GZD" 2>/dev/null
        xdotool windowactivate --sync "$GZD" 2>/dev/null
        sleep 1
        set -- $(wmctrl -lG | awk -v id="$GZ" '$1==id{print $5}')
        [ "${1:-0}" -ge $((SW - 40)) ] && break
        wmctrl -i -r "$GZ" -e "0,0,0,$((SW - 6)),$((SH - 40))"
        sleep 1
    done
    echo ">>> Gazebo: $(wmctrl -lG | awk -v id="$GZ" '$1==id{print $5"x"$6"+"$3"+"$4}')"
fi

if [ "$MODE" != "gazebo" ] && [ -n "$WS" ]; then
    set -- $(wmctrl -lG | awk -v id="$WS" '$1==id{print $5, $6}')
    W=${CW:-$1}; H=${CH:-$2}
    [ -z "$W" ] && W=614
    [ -z "$H" ] && H=460
    WH=$((H + TITLE_H))
    PX=$((SW - W - 8)); PY=$((SH - WH - 28))
    [ "$PX" -lt 0 ] && PX=0
    [ "$PY" -lt 0 ] && PY=0
    for i in 1 2 3; do
        wmctrl -i -r "$WS" -e "0,$PX,$PY,$W,$WH" 2>/dev/null
        wmctrl -i -r "$WS" -b add,above 2>/dev/null
        sleep 1
        set -- $(wmctrl -lG | awk -v id="$WS" '$1==id{print $5, $6, $3, $4}')
        [ "$1" = "$W" ] && [ "$3" = "$PX" ] && break
    done
    echo ">>> 腕部预览: $(wmctrl -lG | awk -v id="$WS" '$1==id{print $5"x"$6"+"$3"+"$4}') (目标 ${W}x${WH}+${PX}+${PY})"
fi

[ -z "$GZ" ] && echo ">>> 没有 Gazebo 窗口(先跑 bash tools/open_gui.sh)"
[ -z "$WS" ] && echo ">>> 没有腕部预览窗口(先跑 bash tools/wrist_view.sh)"
exit 0
