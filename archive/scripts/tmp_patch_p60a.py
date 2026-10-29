#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P60a: 修机器人俯仰失稳(行驶中翘头 31° -> 深度相机看地 -> 全向急停 -> 打转)。

根因(实测):
  * 牵引力作用在轮地接触点(z=0), 重心在 z=0.13 -> 每 1N 牵引产生 0.13Nm
    抬头力矩; 而支撑多边形的前缘就是驱动轮轴(x=0), 重心正好在轮轴上方,
    没有前部支撑点抵抗 -> 前进几秒就翘到 31°(实测 3.8s)。
  * P59 把 caster 球用 fixed joint 焊死 -> 球变成"锁死的轮子", 拖拽力矩
    再加一份, 翘得更快; 去掉 joint 后球又掉出去, 只剩两轮 -> 俯仰中性,
    落地即歪。

修法:
  1) 前后各一个万向轮(caster), 都用 revolute(绕自身轮轴) -> 自由滚动,
     不拖拽; 前轮专门抵抗抬头力矩, 后轮防止后仰。
  2) 重心略前移(x=+0.02) -> 驱动轮承担 ~83% 载荷, 牵引力不打滑。
  3) 轮子轴向沿 y(rpy 1.5707), 与驱动轮一致; 半径 0.03, 接地高度不变。
"""
import io
import xml.etree.ElementTree as ET

SRC = '_work_area_p58.sdf'
DST = '_work_area_p60.sdf'

src = io.open(SRC, encoding='utf-8').read()

# ---------- 1) 重心前移 ----------
OLD_INERT = """        <inertial>
          <mass>4.0</mass>
          <inertia><ixx>0.0167</ixx><iyy>0.0333</iyy><izz>0.0433</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
        </inertial>"""
NEW_INERT = """        <!-- P60: 重心前移 2cm。牵引力作用在地面, 重心抬高会把车头翘起来;
             重���略前于驱动轮轴(但仍在前后万向轮之间), 驱动轮承担 ~83% 载荷,
             牵引不打滑, 同时前万向轮有足够杠杆抵抗抬头力矩。 -->
        <inertial>
          <pose>0.02 0 0 0 0 0</pose>
          <mass>4.0</mass>
          <inertia><ixx>0.0167</ixx><iyy>0.0333</iyy><izz>0.0433</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
        </inertial>"""
assert src.count(OLD_INERT) == 1, src.count(OLD_INERT)
src = src.replace(OLD_INERT, NEW_INERT, 1)

# ---------- 2) 后万向轮: 球 -> 可滚动小轮, fixed joint -> revolute ----------
OLD_CASTER = """      <link name="caster">
        <pose relative_to="chassis">-0.12 0 -0.10 0 0 0</pose>
        <inertial>
          <mass>0.1</mass>
          <inertia><ixx>0.000036</ixx><iyy>0.000036</iyy><izz>0.000036</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
        </inertial>
        <collision name="c">
          <geometry><sphere><radius>0.03</radius></sphere></geometry>
          <surface>
            <friction><ode><mu>0.05</mu><mu2>0.05</mu2></ode></friction>
          </surface>
        </collision>
        <visual name="v">
          <geometry><sphere><radius>0.03</radius></sphere></geometry>
          <material><ambient>0.3 0.3 0.3 1</ambient><diffuse>0.3 0.3 0.3 1</diffuse></material>
        </visual>
      </link>"""
NEW_CASTER = """      <!-- P60: 后万向轮(可自由滚动的小轮)。原来是个球: 无 joint 时掉出去
           变成额外摩擦源; 用 fixed joint 焊死时变成"锁死的轮子", 拖拽力矩
           把车头翘到 31°(实测前进 3.8s 即翘头, 深度相机随即看地, 全向急停
           -> 原地打转 -> 丢失目标)。改成 revolute 绕自身轮轴 -> 自由滚动。 -->
      <link name="caster">
        <pose relative_to="chassis">-0.12 0 -0.10 1.5707 0 0</pose>
        <inertial>
          <mass>0.08</mass>
          <inertia><ixx>0.000030</ixx><iyy>0.000030</iyy><izz>0.000030</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
        </inertial>
        <collision name="c">
          <geometry><cylinder><radius>0.03</radius><length>0.02</length></cylinder></geometry>
          <surface>
            <friction><ode><mu>0.25</mu><mu2>0.10</mu2></ode></friction>
          </surface>
        </collision>
        <visual name="v">
          <geometry><cylinder><radius>0.03</radius><length>0.02</length></cylinder></geometry>
          <material><ambient>0.3 0.3 0.3 1</ambient><diffuse>0.3 0.3 0.3 1</diffuse></material>
        </visual>
      </link>
      <!-- P60: 前万向轮 —— 关键。牵引力在地面、重心在 0.13m, 每 1N 牵引
           产生 0.13Nm 抬头力矩; 原来支撑多边形前缘就是驱动轮轴(重心正好
           在其上), 没有任何前部支撑抵抗它 -> 前进几秒就翘头。前轮补上
           这个支撑点, 翘头力矩由前轮载荷转移抵消。 -->
      <link name="caster_front">
        <pose relative_to="chassis">0.12 0 -0.10 1.5707 0 0</pose>
        <inertial>
          <mass>0.08</mass>
          <inertia><ixx>0.000030</ixx><iyy>0.000030</iyy><izz>0.000030</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
        </inertial>
        <collision name="c">
          <geometry><cylinder><radius>0.03</radius><length>0.02</length></cylinder></geometry>
          <surface>
            <friction><ode><mu>0.25</mu><mu2>0.10</mu2></ode></friction>
          </surface>
        </collision>
        <visual name="v">
          <geometry><cylinder><radius>0.03</radius><length>0.02</length></cylinder></geometry>
          <material><ambient>0.3 0.3 0.3 1</ambient><diffuse>0.3 0.3 0.3 1</diffuse></material>
        </visual>
      </link>"""
assert src.count(OLD_CASTER) == 1, src.count(OLD_CASTER)
src = src.replace(OLD_CASTER, NEW_CASTER, 1)

# ---------- 3) joint: fixed -> revolute(绕轮轴), 并新增前轮 joint ----------
OLD_JOINT = """      <!-- P59: caster 原来没有 joint, gz 把它当自由刚体 -> 受重力掉落,
           球体卡在底盘下方变成额外摩擦源, 机器人实际速度长期低于指令值
           (P40 里怀疑过 /robot/odom 计量, 根因之一其实在这里)。 -->
      <joint name="caster_joint" type="fixed">
        <parent>chassis</parent>
        <child>caster</child>
      </joint>"""
NEW_JOINT = """      <!-- P60: 万向轮改成绕自身轮轴的 revolute, 可自由滚动(不拖拽);
           axis 用 child 系(小轮圆柱轴是 +z, link 的 rpy 1.5707 已把它转到 y)。 -->
      <joint name="caster_joint" type="revolute">
        <parent>chassis</parent>
        <child>caster</child>
        <axis>
          <xyz resolved_frame="child">0 0 1</xyz>
          <limits><lower>-1e16</lower><upper>1e16</upper></limits>
        </axis>
      </joint>
      <joint name="caster_front_joint" type="revolute">
        <parent>chassis</parent>
        <child>caster_front</child>
        <axis>
          <xyz resolved_frame="child">0 0 1</xyz>
          <limits><lower>-1e16</lower><upper>1e16</upper></limits>
        </axis>
      </joint>"""
assert src.count(OLD_JOINT) == 1, src.count(OLD_JOINT)
src = src.replace(OLD_JOINT, NEW_JOINT, 1)

ET.fromstring(src)
io.open(DST, 'w', encoding='utf-8', newline='\n').write(src)
print('P60a OK: caster_joint=%d caster_front_joint=%d inert_pose=%d'
      % (src.count('"caster_joint"'), src.count('"caster_front_joint"'),
         src.count('<pose>0.02 0 0 0 0 0</pose>')))
