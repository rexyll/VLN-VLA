#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P41: 给 person 模型补上 torso_joint / head_joint

现象(用户反馈): 场景里有一个蓝色胶囊状障碍物, 人能穿过它, 机器人识别不到,
             每次都撞上。

实测根因:
  person 模型有 5 个 link —— skirt / torso / head / left_wheel / right_wheel,
  但只定义了 2 个轮子的 joint。torso 和 head **没有任何关节连到 skirt 上**,
  在 gz-sim 里就是两个自由刚体: 开局直接掉下来, 然后在地上漂走。

  运行时抓 /world/follow_world/pose/info 证实:
      skirt  ( 0.00,  0.00, 0.28)   ← 人的本体(暗灰圆柱)还站在原地
      torso  (-10.20, -4.81, 0.79)  ← 蓝色胶囊, 已漂出场外
      head   ( -8.91, -7.31, 0.82)  ← 肤色球, 已漂出场外
  2 秒内 torso 又滑了 0.12m —— 是带碰撞体的自由刚体, 会一直在场里乱滚。

  于是就出现了用户看到的三件事:
    1) 场里多出一个"蓝色胶囊状障碍物" = 掉落的 torso(capsule r0.17 len0.4, 蓝);
    2) "人能穿过它" = person 的 self_collide=false, 且它本来就不在人身上;
    3) "机器人识别不到、每次都撞" = 它躺在地上(顶面约 0.34m), 而 4 台深度
       相机的有效取带是仰角 +1.5°~+18°(下沿刚过车头), 贴地物体只露一条窄边,
       逐扇区 5% 分位很容易被地面回波稀释掉; 而它又是带碰撞的实体, 于是撞上。

  顺带: 因为躯干和头掉了, 场景里的"人"其实只剩一个暗灰圆柱 + 两个轮子,
        person_vision 的 bbox 高度只有 0.44m, 也是它检出率偏低的一个隐性原因。

修法: 补两个 fixed joint。gz-sim 会把 fixed 子 link 合并进父 link(lumping),
      质量变成 3+3+1=7kg, 差速插件的 max_linear_acceleration 0.9 仍然够用。
"""
import io
import os
import sys

P = os.environ.get('SDF_SRC') or '_p41_work_area.sdf'
src = io.open(P, encoding='utf-8').read()

ANCHOR = """      <plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">
        <left_joint>left_wheel_joint</left_joint>
        <right_joint>right_wheel_joint</right_joint>
        <wheel_separation>0.40</wheel_separation>"""

ADD = """      <!-- P41: 原来 torso / head 只有 link 定义、没有 joint, 在 gz-sim 里
           就是两个自由刚体 —— 开局从人身上掉下来, 然后在地上漂着乱滚。
           表现出来就是"场里凭空多了一个蓝色胶囊障碍物, 人能穿过去, 机器人
           却识别不到、每次都撞"。这里补上 fixed joint 让它们真正长在身上。 -->
      <joint name="torso_joint" type="fixed">
        <parent>skirt</parent>
        <child>torso</child>
      </joint>
      <joint name="head_joint" type="fixed">
        <parent>skirt</parent>
        <child>head</child>
      </joint>
      <plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">
        <left_joint>left_wheel_joint</left_joint>
        <right_joint>right_wheel_joint</right_joint>
        <wheel_separation>0.40</wheel_separation>"""

if 'torso_joint' in src:
    print('已存在, 跳过')
    sys.exit(0)
if src.count(ANCHOR) != 1:
    print('!! 锚点匹配 %d 次' % src.count(ANCHOR))
    sys.exit(1)
src = src.replace(ANCHOR, ADD)
io.open(P, 'w', encoding='utf-8', newline='\n').write(src)

seg = src[src.index('<model name="person"'):]
print('P41 OK  person 模型 joint 数 = %d' % seg.count('<joint '))
for j in ('torso_joint', 'head_joint', 'left_wheel_joint', 'right_wheel_joint'):
    print('   %-18s %s' % (j, 'OK' if j in seg else '缺失'))
