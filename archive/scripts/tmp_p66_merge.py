#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tmp_p66_merge.py —— P66 最终修复: person 的 torso/head 合并进 skirt 单一 link

原因: ①torso/head 无 joint → 自由体掉落(T6); ②加 fixed joint 后 gz server
     在世界加载后不步进(实测 /stats 无数据、人冻结在原点) —— fixed joint
     结构在本环境触发死锁。
方案: 单一 link 多 collision/visual 元素 —— 刚体不可能掉落, 无 joint 无死锁。
质量/惯量: 三体合并 m=7.0, 绕 skirt 原点平行轴合成 Ixx=Iyy≈2.255, Izz≈0.152。
"""
import re

p = '/home/earl/workspace/VLA/vla_ws/src/vla_follow/worlds/work_area.sdf'
s = open(p, encoding='utf-8').read()

new_person = '''    <!-- "人": 人形外观(裙摆/躯干/头部合并为单一刚体 link, 物理上不可能掉落)
         + 隐藏轮组差速驱动, 初始位于机器人前方 1m -->
    <model name="person" canonical_link="skirt">
      <pose>1 0 0 0 0 0</pose>
      <self_collide>false</self_collide>
      <link name="skirt">
        <pose>0 0 0.28 0 0 0</pose>
        <inertial>
          <mass>7.0</mass>
          <!-- 三体(skirt+torso+head)平行轴合成 -->
          <inertia><ixx>2.255</ixx><iyy>2.255</iyy><izz>0.152</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
        </inertial>
        <!-- 碰撞体直达近地面: 防止机器人钻入裙底被卡死 -->
        <collision name="c_skirt">
          <pose>0 0 -0.04 0 0 0</pose>
          <geometry><cylinder><radius>0.26</radius><length>0.44</length></cylinder></geometry>
          <surface><friction><ode><mu>0.1</mu><mu2>0.1</mu2></ode></friction></surface>
        </collision>
        <visual name="v_skirt">
          <pose>0 0 -0.04 0 0 0</pose>
          <geometry><cylinder><radius>0.26</radius><length>0.44</length></cylinder></geometry>
          <material><ambient>0.25 0.25 0.28 1</ambient><diffuse>0.25 0.25 0.28 1</diffuse></material>
        </visual>
        <!-- 躯干(胶囊, 相对 skirt 中心上方 0.55m) -->
        <collision name="c_torso">
          <pose>0 0 0.55 0 0 0</pose>
          <geometry><capsule><radius>0.17</radius><length>0.4</length></capsule></geometry>
        </collision>
        <visual name="v_torso">
          <pose>0 0 0.55 0 0 0</pose>
          <geometry><capsule><radius>0.17</radius><length>0.4</length></capsule></geometry>
          <material><ambient>0.18 0.44 0.82 1</ambient><diffuse>0.18 0.44 0.82 1</diffuse></material>
        </visual>
        <!-- 头部(球, 相对 skirt 中心上方 1.05m) -->
        <collision name="c_head">
          <pose>0 0 1.05 0 0 0</pose>
          <geometry><sphere><radius>0.13</radius></sphere></geometry>
        </collision>
        <visual name="v_head">
          <pose>0 0 1.05 0 0 0</pose>
          <geometry><sphere><radius>0.13</radius></sphere></geometry>
          <material><ambient>0.94 0.78 0.63 1</ambient><diffuse>0.94 0.78 0.63 1</diffuse></material>
        </visual>
      </link>
      <link name="left_wheel">
        <pose relative_to="skirt">0 0.20 -0.225 1.5707 0 0</pose>
        <inertial>
          <mass>0.25</mass>
          <inertia><ixx>0.0003</ixx><iyy>0.0003</iyy><izz>0.0003</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
        </inertial>
        <collision name="c">
          <geometry><cylinder><radius>0.055</radius><length>0.05</length></cylinder></geometry>
          <surface><friction><ode><mu>1.2</mu><mu2>1.0</mu2></ode></friction></surface>
        </collision>
        <visual name="v">
          <geometry><cylinder><radius>0.055</radius><length>0.05</length></cylinder></geometry>
          <material><ambient>0.1 0.1 0.1 1</ambient><diffuse>0.1 0.1 0.1 1</diffuse></material>
        </visual>
      </link>
      <link name="right_wheel">
        <pose relative_to="skirt">0 -0.20 -0.225 1.5707 0 0</pose>
        <inertial>
          <mass>0.25</mass>
          <inertia><ixx>0.0003</ixx><iyy>0.0003</iyy><izz>0.0003</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
        </inertial>
        <collision name="c">
          <geometry><cylinder><radius>0.055</radius><length>0.05</length></cylinder></geometry>
          <surface><friction><ode><mu>1.2</mu><mu2>1.0</mu2></ode></friction></surface>
        </collision>
        <visual name="v">
          <geometry><cylinder><radius>0.055</radius><length>0.05</length></cylinder></geometry>
          <material><ambient>0.1 0.1 0.1 1</ambient><diffuse>0.1 0.1 0.1 1</diffuse></material>
        </visual>
      </link>
      <joint name="left_wheel_joint" type="revolute">
        <parent>skirt</parent>
        <child>left_wheel</child>
        <axis>
          <xyz resolved_frame="child">0 0 1</xyz>
          <limits><lower>-1e16</lower><upper>1e16</upper></limits>
        </axis>
      </joint>
      <joint name="right_wheel_joint" type="revolute">
        <parent>skirt</parent>
        <child>right_wheel</child>
        <axis>
          <xyz resolved_frame="child">0 0 1</xyz>
          <limits><lower>-1e16</lower><upper>1e16</upper></limits>
        </axis>
      </joint>
      <plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">
        <left_joint>left_wheel_joint</left_joint>
        <right_joint>right_wheel_joint</right_joint>
        <wheel_separation>0.40</wheel_separation>
        <wheel_radius>0.055</wheel_radius>
        <topic>/model/person/cmd_vel</topic>
        <odom_topic>/model/person/odometry</odom_topic>
        <odom_publish_frequency>30</odom_publish_frequency>
        <max_linear_acceleration>0.9</max_linear_acceleration>
        <min_linear_acceleration>-0.9</min_linear_acceleration>
        <max_angular_acceleration>2.5</max_angular_acceleration>
        <min_angular_acceleration>-2.5</min_angular_acceleration>
      </plugin>
    </model>
'''

# 定位 person 模型段并整体替换
m = re.search(r'    <!-- "人".*?</model>\n', s, flags=re.S)
if m is None:
    m = re.search(r'    <model name="person".*?</model>\n', s, flags=re.S)
assert m, 'person model block not found'
s = s[:m.start()] + new_person + s[m.end():]
open(p, 'w', encoding='utf-8').write(s)
print('person 模型已重写为单一刚体 link(躯干/头部合并)')
