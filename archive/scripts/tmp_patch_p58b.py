#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P58b: 新增配置项(需求 1/4/5/6 的可调参数)。"""
import io

P = '_p58_follower.py'
s = io.open(P, encoding='utf-8').read()

OLD = "    'scan_rounds': 2,               # 最多\"推测→前往→扫描\"轮次\n}\n"
assert s.count(OLD) == 1, s.count(OLD)

NEW = """    'scan_rounds': 2,               # 最多"推测→前往→扫描"轮次
    # ==== P58: 需求 1~6 的可调参数 ====
    # 需求1/2: 目标必须压在 front 相机水平中线 ±ao_center_deg 内。
    # 10° 是硬指标; front 深度相机半视场 51.5°, 留 5.5° 余量取 46° 作
    # "还在视野里"的判据 —— 检测框在画面边缘时 bbox 会被裁掉一半, 深度
    # 取窗也会偏, 不能拿 51.5° 当边界用。
    'ao_center_deg': 10.0,
    'ao_front_fov_deg': 46.0,
    # 边走边转时的速度上限: 越小越不容易把目标甩出视野, 但越容易掉队。
    'ao_steer_vmax': 0.46,
    # 需求2/3: 局部路径规划(栅格 A*)。关掉则退回"距离环 + 安全层"。
    'use_planner': True,
    'plan_period_s': 0.35,          # 重规划周期
    'plan_goal_move_m': 0.30,       # 目标移动超过此值立即重规划
    'plan_lookahead_m': 0.55,       # 纯追踪前视距离
    'plan_v_max': 0.55,             # 路径跟踪速度上限
    # 需求4: 与障碍表面的最小间隙(默认 0.2m, 配置文件可直接改)。
    # 注意口径: 这是**扣除车体外接半径之后**的表面间隙, 与评测脚本的
    # min_gap 完全一致, 不是"车心到障碍中心"的距离。
    'obstacle_clear_m': 0.20,
    # 需求5/6: 与目标(人)的最小距离 + 进入减速区的距离
    'person_keep_m': 0.70,
    'person_slow_m': 1.60,
}
"""
s = s.replace(OLD, NEW, 1)

io.open(P, 'w', encoding='utf-8', newline='\n').write(s)
print('cfg ok')
