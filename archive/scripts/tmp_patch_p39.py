#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tmp_patch_p39.py —— P39: 关闭航点绕障(死锁源) + 人速匹配

第六轮日志(eval6):
  `绕障0左 → 绕障0右 → 绕障0左 → 绕障0右` 每 ~1s 翻一次,
  同时 `指令v=0.00 实际v=0.02`, 机器人卡在柱子旁原地打转, 人从 5m
  跑到 7m。根因是 _waypoint() 每 tick 重算切点, 障碍恰好在
  "左绕/右绕"临界角上时切点两侧来回跳, 而绕障分支 `abs(err_yaw)>2.2`
  直接把 v 置 0 并强制转 —— 形成死锁。

  安全层(_safety_gate)本身已经把避障做到 min_gap=+0.333 / 0 碰撞,
  它的"停-转-走"行为比航点绕障更稳。因此默认关闭航点绕障。

  V1  新增 use_waypoint=False, 绕障航点默认不启用(保留代码可回溯)
  V2  人速上限 0.50 → 0.40 —— 机器人 approach_speed=0.52(真实),
      对正段只有 0.44, 人走 0.50 时追不上; 0.40 留出 0.12 余量,
      速度变化仍是 0.12~0.40(3.3 倍), "时快时慢"足够明显
"""
import io
import json
import sys

FP = 'vla_ws/src/vla_follow/vla_follow/follower.py'
WP = 'vla_ws/src/vla_follow/vla_follow/person_walk.py'
JP = 'vla_demo/vision_config.json'

fs = io.open(FP, encoding='utf-8').read()
ws = io.open(WP, encoding='utf-8').read()
if 'P39' in fs:
    print('!! 已打过 P39')
    sys.exit(1)


def rep(buf, old, new, tag):
    n = buf.count(old)
    if n != 1:
        print('!! [%s] 锚点命中 %d 次' % (tag, n))
        sys.exit(1)
    print('   ok %s' % tag)
    return buf.replace(old, new, 1)


# V1a 配置
fs = rep(fs, """    'cmd_gain': 1.62,               # P36: 指令速度→实际速度的补偿(实测×0.62)""",
         """    'cmd_gain': 1.62,               # P36: 指令速度→实际速度的补偿(实测×0.62)
    'use_waypoint': False,          # P39: 航点绕障在临界角上会左右翻转死锁""",
         'V1a')

# V1b 短路航点
fs = rep(fs, """        tx, ty, note, dwp = self._waypoint(rx, ry, px, py)
        if note:""",
         """        tx, ty, note, dwp = self._waypoint(rx, ry, px, py)
        if not self.cfg['use_waypoint']:
            # P39: 航点绕障在"障碍恰好在左/右绕临界角"时切点两侧来回跳,
            # 而绕障分支会把 v 置 0 强制转向 → 机器人原地打转卡死。
            # 安全层的"停-转-走"已能保证 0 碰撞, 这里直接不启用航点。
            note = ''
        if note:""",
         'V1b')
io.open(FP, 'w', encoding='utf-8', newline='\n').write(fs)

# V2 人速
ws = rep(ws, """        # P36: 0.55→0.50, 给速度环留出真正的跟踪余量
        self.target_speed = random.uniform(0.15, 0.50)""",
         """        # P36: 0.55→0.50, 给速度环留出真正的跟踪余量
        # P39: 0.50→0.40 —— 对正段只有 0.85*0.52=0.44, 人走 0.50 追不上
        self.target_speed = random.uniform(0.12, 0.40)""",
         'V2a')
ws = rep(ws, """                self.target_speed = random.uniform(0.15, 0.50)""",
         """                self.target_speed = random.uniform(0.12, 0.40)""",
         'V2b')
io.open(WP, 'w', encoding='utf-8', newline='\n').write(ws)

cfg = json.loads(io.open(JP, encoding='utf-8').read())
cfg['use_waypoint'] = False
io.open(JP, 'w', encoding='utf-8', newline='\n').write(
    json.dumps(cfg, ensure_ascii=False, indent=1))

print('P39_APPLIED')
