#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tmp_patch_p35.py —— P35: 测距偏置补偿 + 速度匹配 + 侧翼贴死不前进

标定结论(实测 1.0~5.0m, 8 个点):
    vis_dist = 真值距离 - 0.25m   (偏差 -0.273 ~ -0.222, 极稳定)
  原因: 视觉测的是人体表面(躯干半径 0.26), 而真值是模型中心。
  于是 target_d=1.0 实际对应"中心距 0.75m" —— 机器人一直在贴身跟,
  实测开局 d=0.53/0.49, 并因此把"人"挤到 person_walk 的让开阈值。

  T1  新增 vis_bias_m=0.25, 视觉距离统一补偿成"中心距", target_d 仍是 1.0
  T2  approach_speed 0.38 → 0.52, 让人走 0.55 时机器人有速度余量
      (原 0.38 追 0.55 的人, 实测 45s 内距离从 0.5m 拉到 5.4m)
  T3  侧翼贴死时不再"限速 0.10 继续挤", 改为停止前进 + 强制往空侧转
      (0.10m/s × 15s = 1.5m, 足够钻进立柱: 实测 gap 到 -0.42)
  T4  人速上限 0.60 → 0.55, 与机器人 0.52 留出跟踪余量
"""
import io
import re
import sys

FP = 'vla_ws/src/vla_follow/vla_follow/follower.py'
WP = 'vla_ws/src/vla_follow/vla_follow/person_walk.py'
JP = 'vla_demo/vision_config.json'

s = io.open(FP, encoding='utf-8').read()
w = io.open(WP, encoding='utf-8').read()
if 'P35' in s:
    print('!! 已打过 P35')
    sys.exit(1)


def rep(buf, old, new, tag):
    n = buf.count(old)
    if n != 1:
        print('!! [%s] 锚点命中 %d 次' % (tag, n))
        sys.exit(1)
    print('   ok %s' % tag)
    return buf.replace(old, new, 1)


# ---- T1a: 配置 ----
s = rep(s, """    'approach_speed': 0.38,""",
        """    'approach_speed': 0.52,         # P35: 人速上限 0.55, 需留跟踪余量
    'vis_bias_m': 0.25,             # P35: 视觉测的是人体表面, 补成模型中心距
                                    #      (实测 1~5m 偏差 -0.25 极稳)""",
        'T1a')

# ---- T1b: 视觉距离补偿 ----
s = rep(s, """            vis_cam = str(vis.get('cam', 'front'))
            bearing = float(vis['bearing'])     # 相对机器人前向, + = 左
            vdist = float(vis['dist'])""",
        """            vis_cam = str(vis.get('cam', 'front'))
            bearing = float(vis['bearing'])     # 相对机器人前向, + = 左
            # P35: 相机测的是人体表面, 加偏置换算成"人中心到机器人中心"
            vdist = float(vis['dist']) + float(self.cfg['vis_bias_m'])""",
        'T1b')

# ---- T3: 侧翼贴死 ----
s = rep(s, """                d_wing = min(d_l, d_r, _pv('bl'), _pv('br'))
                if d_wing < stop_d:
                    v_t = min(v_t, 0.10)                # 侧翼已贴死, 别再挤""",
        """                d_wing = min(d_l, d_r, _pv('bl'), _pv('br'))
                if d_wing < stop_d:
                    # P35: 原实现"限速到 0.10 继续挤"在实测中会慢慢钻进
                    # 立柱(0.10m/s × 15s = 1.5m, gap 走到 -0.42)。侧翼已
                    # 进入停车距离时必须停止前进, 并强制往空的一侧转。
                    v_t = 0.0
                    if abs(w_t) < 0.35:
                        w_t = 0.9 if d_r < d_l else -0.9""",
        'T3')

io.open(FP, 'w', encoding='utf-8', newline='\n').write(s)

# ---- T4: 人速上限 ----
w = rep(w, """        # P33: 速度幅值拉开(原 0.20~0.45 变化不明显, 体现不出"时快时慢")
        self.target_speed = random.uniform(0.15, 0.60)""",
        """        # P33: 速度幅值拉开(原 0.20~0.45 变化不明显, 体现不出"时快时慢")
        # P35: 上限 0.60→0.55 —— 机器人 approach_speed=0.52, 人要能被跟上
        self.target_speed = random.uniform(0.15, 0.55)""",
        'T4a')
w = rep(w, """            if random.random() < 0.6:
                self.target_speed = random.uniform(0.15, 0.60)""",
        """            if random.random() < 0.6:
                self.target_speed = random.uniform(0.15, 0.55)""",
        'T4b')
io.open(WP, 'w', encoding='utf-8', newline='\n').write(w)

# ---- 配置文件同步 ----
import json
cfg = json.loads(io.open(JP, encoding='utf-8').read())
cfg['approach_speed'] = 0.52
cfg['vis_bias_m'] = 0.25
io.open(JP, 'w', encoding='utf-8', newline='\n').write(
    json.dumps(cfg, ensure_ascii=False, indent=1))

print('P35_APPLIED')
