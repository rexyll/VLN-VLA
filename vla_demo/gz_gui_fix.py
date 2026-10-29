#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gz_gui_fix.py —— 每次启动 gz 前修正 GUI 配置

背景: gz 的 GUI 在**正常退出(SIGTERM)**时会把当前布局和相机位姿写回
~/.gz/sim/8/gui.config, 于是任何手工修改都会被下一次退出覆盖。实测:
改了 camera_pose=13m 俯视, 重启后仍旧是上次那个斜视近景。

所以放在 start_demo.sh 里、`gz sim` 之前调用, 每次启动都重新写一遍:
  1) MinimalScene 相机改成全场俯视(12m 场地, z=13 刚好铺满)
  2) 渲染引擎写成 ogre —— 与 --render-engine ogre 一致(本机 VMware 显卡
     用不了 ogre2, 回落时相机配置会失效)
  3) 去掉 EntityTree / ComponentInspector 两块右侧面板
"""
import os
import re

CFG = os.path.expanduser('~/.gz/sim/8/gui.config')
# 12m 场地: z=13 刚好铺满。pitch 用 **+1.45**(正角!), 不是 -1.45 ——
# MinimalScene 的 camera_pose 与 camera sensor 同一套朝向约定(+X 为前,
# 绕 Y 正转才把 +X 转到 -Z 朝下)。写负角相机朝上看天, 3D 视图就只剩天蓝色,
# 而"还剩底色"极具误导性, 会被当成 GUI GL 崩了。抬 5° 是为了不落在正好垂直的
# 奇异位形上, 顺带能看出障碍高度。
CAM = '0.0 0.0 13.0 0 1.45 0'

if not os.path.exists(CFG):
    print('[GUI-FIX] 无 %s, 跳过' % CFG)
    raise SystemExit(0)

s = open(CFG, encoding='utf-8').read()
orig = s

# 1) 相机
if '<camera_pose>' in s:
    s = re.sub(r'<camera_pose>.*?</camera_pose>',
               '<camera_pose>%s</camera_pose>' % CAM, s, count=1)
else:
    s = s.replace('</plugin>', '</plugin>', 1)

# 2) 渲染引擎
s = s.replace('<engine>ogre2</engine>', '<engine>ogre</engine>')

# 2b) 图像窗口: 本机 VMware(SVGA3D, glxinfo 报 Accelerated: no / Video memory 1MB)
#     上 GUI 的 **3D 视图**会整块不渲染(只剩底色), 而服务器端相机渲染一直正常。
#     所以这个 ImageDisplay 窗口默认订阅 **/overhead/image**(世界里 overhead_cam
#     的俯视相机) —— Qt 贴图不经过 3D 视图, **一定出图**; 而且这张图上就有全部
#     需要看的东西: 半径 0.66 的红色急停盘、1.60 的黄色虚线限速圈、青/红轨迹点、
#     品红 Nav2 规划路径 —— 正好用来判断"是否按规划走 / 是否在障碍旁徘徊"。
#     想看机器人自己的视角: 在该窗口顶部 topic 下拉里切到 **/camera**。
#     只保留**一个** ImageDisplay: 挂两个实测会互相抢 image provider
#     (QML 报 Invalid image provider, 第二个窗口空白且不订阅话题)。
VIEW = """<plugin filename="ImageDisplay" name="Camera View">
    <gz-gui>
        <property type="bool" key="showTitleBar">true</property>
        <property type="bool" key="resizable">false</property>
        <property type="double" key="width">400</property>
        <property type="double" key="height">400</property>
        <property type="double" key="z">2</property>
        <property type="string" key="state">floating</property>
        <anchors target="3D View">
            <line own="left" target="left"/>
            <line own="top" target="top"/>
        </anchors>
    </gz-gui>
    <topic>/overhead/image</topic>
    <height>380</height>
    <width>380</width>
</plugin>"""


def _repl_view(m):
    return VIEW


# 清掉之前试过的第二个 ImageDisplay(会和 Camera View 抢 provider)
s = re.sub(r'\n?<!-- P76b 顶视图.*?name="Overhead View">.*?</plugin>\n?',
           '\n', s, flags=re.S)
if 'name="Camera View"' in s:
    s = re.sub(r'<plugin filename="ImageDisplay" name="Camera View">.*?</plugin>',
               _repl_view, s, count=1, flags=re.S)
else:
    s = s.rstrip('\n') + '\n\n' + VIEW + '\n'

# 3) 面板: 不删! 实测把 EntityTree/ComponentInspector 去掉后, 停靠区失去
#    参照, 3D 视图会塌成左下角一小块(state=docked 需要停靠上下文)。
#    保留默认布局, 用户自己拖分隔条即可。

# 4) P76c: 禁掉 CameraTracking 插件。
#    world 里写了 <gui><camera name="user_cam"><view_controller>orbit</...>,
#    配合 CameraTracking 后, 只要 EntityTree 里选中 robot(启动时常见),
#    相机就**跟着机器人跑并越缩越近**: 实测同一实例 25s / 50s / 90s 三帧
#    分别是 13m 全景 → 斜视近景 → 更近, 完全没法判断"是否按规划路径走"。
#    去掉该插件后相机固定为 <camera_pose>(0 0 13 -1.5708), 机器人/轨迹在
#    画面里移动 —— 这才是看路径该有的视角。
s = re.sub(r'<plugin filename="CameraTracking".*?</plugin>\s*',
           '', s, flags=re.S)

if s != orig:
    open(CFG, 'w', encoding='utf-8').write(s)
    print('[GUI-FIX] 已重写 %s' % CFG)
else:
    print('[GUI-FIX] 无需改动')
