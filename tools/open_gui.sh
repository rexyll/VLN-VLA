#!/bin/bash
# open_gui.sh —— 给"已经在跑"的 gz 服务端挂一个 GUI 窗口, 并把机位调好
#
# 为什么单独一个脚本:
#   * 服务器端是以 `gz sim -s`(无 GUI) 起的, 要画面就得再起一个 GUI 客户端
#     `gz sim -g`, 它通过 gz transport 连上同一个世界, 不会重复加载物理。
#   * gz 的 GUI **正常退出(SIGTERM)时会把布局和相机写回** ~/.gz/sim/8/gui.config,
#     所以改配置前必须先 **-9 硬杀**, 否则刚写的机位立刻被覆盖(这个坑 P76 踩过)。
#   * 本机是 VMware SVGA II, glxinfo 报 direct rendering: Yes 但实际是软件光栅,
#     ogre2 会黑屏/不渲染, 必须回落 ogre(与 start_scene.sh 的 --render-engine 一致)。
#   * 只保留**一个** ImageDisplay: 挂两个会互相抢 image provider, 第二个永远空白。
#
# 用法:
#   bash tools/open_gui.sh                 # 默认 close 机位(看机械臂/灵巧手)
#   bash tools/open_gui.sh wide            # 全场机位(机器人 + 桌子)
#   bash tools/open_gui.sh top             # 俯视全景(看导航轨迹)
#   bash tools/open_gui.sh close /overhead/image   # 指定机位 + 图像窗口话题
#   BIG=0 bash tools/open_gui.sh wide              # 保留 Inspector/Entity tree
#
# 机位约定(gz MinimalScene 的 <camera_pose> = x y z roll pitch yaw, 沿 +X 看):
#   pitch 为**正**表示低头向下看(写负角会朝天, 画面只剩天蓝色, 极易误判成黑屏)。
#
# 2026-10-28(P99 之后): 加 **BIG=1 布局** —— 默认的 GUI 里 Inspector +
# Entity tree 两块 dock 要吃掉右侧 1/3 宽度, 而且 <window> 记的是 1000x845
# (比本机 1280x800 的屏还高), 结果是"窗口下半截出屏、3D 视图只剩巴掌大"。
# 看机器人动作时这两块一点用没有, 于是把它们改成 floating + 2x2 像素(等于
# 收起), 窗口按屏幕实际分辨率铺满。BIG=0 可退回原布局。
#
# 腕部相机不放进这个窗口: gui.config 里根本没有 ImageDisplay 插件(见
# tools/open_gui.sh 的那条注释), 而且同一进程挂两个会抢 image provider。
# 腕部两路 RGB-D 用 `python3 tools/wrist_view.py` 单开一个窗口更清楚。
export DISPLAY="${DISPLAY:-:0}"
CFG="$HOME/.gz/sim/8/gui.config"
PRESET="${1:-close}"
TOPIC="${2:-/drink_cam/image}"
BIG="${BIG:-1}"
SCREEN=$(xdpyinfo 2>/dev/null | awk '/dimensions:/{print $2; exit}')
SW=${SCREEN%x*}; SH=${SCREEN#*x}
[ -z "$SW" ] && SW=1280
[ -z "$SH" ] && SH=800

case "$PRESET" in
  # 0.75,-2.05,1.70 处、离机器人约 2.1m、俯角 22°, 同时框住躯干+双臂+双手
  close) CAM="0.75 -2.05 1.70 0 0.3795 1.713" ;;
  # 4.2m 外看全景: 原点机器人 + (2.6,0.35) 的桌子 + 接水位
  wide)  CAM="1.00 -3.60 2.30 0 0.3547 1.4813" ;;
  # P76 原来的俯视全景
  top)   CAM="0.0 0.0 13.0 0 1.45 0" ;;
  # P87 倒水作业特写: 站在桌子 -y 侧 1.1m 处看杯口(与 drink_cam 同机位附近)
  pour)  CAM="3.15 -0.30 1.22 0 0.2394 2.4512" ;;
  # 看机器人侧后方的"双手捧杯"特写(机器人朝 +x, 从 -y 侧看正面)
  hands) CAM="1.85 -1.05 1.25 0 0.10 1.5708" ;;
  # 正对"人"(spawn 在 (1,0,0), 面朝 +x): 相机站到人前方 1.3m 回看 -x。
  # 用于核对"人的双手是否长在胸口前方"这类静态模型问题 ——
  # 从机器人机位斜看过去手臂只有 0.13m 长, 肉眼极易判错。
  person) CAM="2.30 0.00 1.05 0 0 3.1416" ;;
  # 看人对准桌子的侧脸(从 -y 侧看, 手臂前伸的姿态最清楚)
  pside)  CAM="1.05 -2.10 1.20 0 0.08 1.5708" ;;
  # P98: 自由视角 —— 与 wide 同起点, 但**摘掉图像小窗**(它浮在 3D 视图
  # 左上角, 吃掉一块拖动区域), 起手后左键拖动=环绕、中键=平移、滚轮=缩放
  free)   CAM="1.00 -3.60 2.30 0 0.3547 1.4813" ; FREE=1 ;;
  *)     CAM="$PRESET" ;;
esac
FREE="${FREE:-0}"

# ---- 1) 硬杀旧 GUI(必须 -9, 否则它会写回配置)
pkill -9 -f 'gz sim -g' 2>/dev/null
sleep 1

# ---- 2) 改配置
python3 - "$CFG" "$CAM" "$TOPIC" "$FREE" "$BIG" "$SW" "$SH" <<'EOF'
import re
import sys

cfg, cam, topic, free, big, sw, sh = sys.argv[1:8]
sw, sh = int(sw), int(sh)
s = open(cfg, encoding='utf-8').read()
orig = s
if '<camera_pose>' in s:
    s = re.sub(r'<camera_pose>.*?</camera_pose>',
               '<camera_pose>%s</camera_pose>' % cam, s, count=1)
s = s.replace('<engine>ogre2</engine>', '<engine>ogre</engine>')
if big == '1':
    # <window> 里的宽度/高度是**裸标签**(带属性的那些是 gz-gui property),
    # 所以用 `<width>数字</width>` 精确匹配不会误伤 property key="width"。
    s = re.sub(r'<width>\d+</width>', '<width>%d</width>' % (sw - 6), s, count=1)
    s = re.sub(r'<height>\d+</height>', '<height>%d</height>' % (sh - 40),
               s, count=1)
    # 两块 dock 收起: state -> floating, 尺寸给 2x2 像素
    for nm in ('Component inspector', 'Entity tree'):
        m = re.search(r'<plugin filename="[^"]+" name="%s">.*?</plugin>'
                      % re.escape(nm), s, flags=re.S)
        if not m:
            continue
        blk = m.group(0)
        nb = re.sub(r'(<property [^>]*key="state"[^>]*>)[^<]*(</property>)',
                    r'\1floating\2', blk, count=1)
        nb = nb.replace(
            '<gz-gui>', '<gz-gui>\n'
            '        <property type="double" key="width">2</property>\n'
            '        <property type="double" key="height">2</property>', 1)
        s = s.replace(blk, nb, 1)
# 图像窗口: 只改已有那个 ImageDisplay 的话题, 不新增(两个会抢 provider)
if free == '1':
    # 自由视角: 整块摘掉 ImageDisplay(浮窗占位), 全屏留给 3D 视图
    s = re.sub(r'<!-- 前置相机画面小窗.*?<plugin filename="ImageDisplay"'
               r' name="Camera View">.*?</plugin>\n?', '', s, count=1,
               flags=re.S)
if 'name="Camera View"' in s:
    def fix(m):
        blk = m.group(0)
        if '<topic>' in blk:
            return re.sub(r'<topic>[^<]*</topic>',
                          '<topic>%s</topic>' % topic, blk, count=1)
        return blk.replace('</plugin>', '    <topic>%s</topic>\n</plugin>'
                           % topic, 1)
    s = re.sub(r'<plugin filename="ImageDisplay" name="Camera View">.*?</plugin>',
               fix, s, count=1, flags=re.S)
if s != orig:
    open(cfg, 'w', encoding='utf-8').write(s)
    print('[GUI] 机位 = %s' % cam)
    if 'name="Camera View"' in s:
        print('[GUI] 图像窗口 = %s' % topic)
    else:
        print('[GUI] 本配置无 ImageDisplay 插件 —— 腕部画面看 '
              'python3 tools/wrist_view.py')
    if big == '1':
        print('[GUI] 布局 = 3D 视图铺满 (Inspector / Entity tree 已收起)')
else:
    print('[GUI] 配置无需改动')
EOF

# ---- 3) 起 GUI 客户端
mkdir -p "$HOME/workspace/VLA/vla_ws/logs"
LOG="$HOME/workspace/VLA/vla_ws/logs/gzgui.log"
setsid nohup gz sim -g > "$LOG" 2>&1 < /dev/null &
sleep 8
n=$(ps -eo args --no-headers | grep -c '[g]z sim -g')
err=$(grep -c 'Unable to open\|Failed to create' "$LOG" 2>/dev/null)
echo ">>> GUI 进程数: $n   启动错误: $err"
[ "$err" != "0" ] && tail -5 "$LOG"
# 窗口铺满(mutter 下 config 里的 <window> 只是"上次关闭时的尺寸", 不保证生效;
# 而且 WM 会在几秒后把只设过一次的几何摆回去, 所以交给 gui_arrange.sh 反复校验)
if [ "$BIG" = "1" ]; then
  wmctrl -l 2>/dev/null | head -4
  bash "$HOME/workspace/VLA/tools/gui_arrange.sh" gazebo
fi
if [ "$FREE" = "1" ]; then
    echo ">>> 自由视角已就绪: 左键拖动=环绕旋转, 中键拖动=平移, 滚轮=缩放"
else
    echo ">>> 窗口已在 DISPLAY=$DISPLAY 上打开; 3D 视图里鼠标左键拖动=旋转, 滚轮=缩放"
fi
