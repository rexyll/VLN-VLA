#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P47 —— 绕障抢占 ALIGN + 视觉跳变抑制。

P46 加了锁存式绕障, 但实测完全没触发。原因:
  `if self._aligning:` 那段在 891 行 **直接 return**, 而绕障在 900+ 行。
  人站在障碍后面时 front 相机被遮, 检出退化成 left/right 的残缺/伪检,
  brg 一跳到 ±50~120° 就 `_aligning = True` → 原地转 → return,
  绕障永远执行不到 → 永久"对正→转→再对正"。

同时实测发现视觉在"人贴着障碍"时非常不稳:
  vis_d 在 0.28~1.74 之间摆(6 倍), 主相机在 left/right 之间来回切。
  这些跳变直接传进控制环, 让 brg 乱抖。

本补丁:
 A. 视觉距离/方位做"时空一致性"抑制: 单帧跳变沿用上一帧(最多 0.6s),
    跨相机的真实切换(dist 变化小)不抑制。
 B. 把锁存式绕障搬到 ALIGN 之前并抢占: 一旦在绕障态, 直接接管速度
    通道并 return, 不再进 ALIGN。
"""
import io
import os
import re
import sys

SRC = os.environ.get('VLA_SRC', '/home/earl/workspace/VLA/vla_ws/src/'
                     'vla_follow/vla_follow/follower.py')
s = io.open(SRC, encoding='utf-8').read()
n = 0


def sub(old, new, tag, cnt=1):
    global s, n
    if s.count(old) != cnt:
        print('!! %s 锚点 count=%d' % (tag, s.count(old)))
        sys.exit(2)
    s = s.replace(old, new, cnt)
    n += 1
    print('   ok %s' % tag)


# ============ A: 滤波状态 ============
sub("""        self._lost_announced = False  # 是否已提示"目标丢失\"""",
    """        self._lost_announced = False  # 是否已提示"目标丢失"
        # P47: 视觉时空一致性滤波
        self._fdist = None            # 滤波后的目标距离
        self._fbrg = None             # 滤波后的目标方位(相对车头)
        self._fhold_until = 0.0       # 跳变抑制截止时刻
        self._fcam = ''               # 上一帧主相机""",
    'A1 滤波状态')

# ============ B: 滤波逻辑 ============
sub("""            px = rx + math.cos(ryaw + bearing) * vdist
            py = ry + math.sin(ryaw + bearing) * vdist
            self._last_vis_xy = (px, py)      # 只由视觉刷新""",
    """            # P47: 时空一致性抑制。人贴着障碍时 bbox 只剩残缺的一小块,
            # 深度取窗还会混进障碍 —— 实测 vis_d 在 0.28~1.74 之间摆 6 倍,
            # 主相机在 left/right 之间来回切, brg 跟着跳到 ±50~120°。
            # 这种跳变一旦进控制环, 就会把机器人打进"对正→原地转"的死循环。
            # 判据: 单帧距离跳 >0.45m 或方位跳 >35°, 视为不可信, 沿用上一帧;
            #       但最多抑制 0.6s, 超时后强制接受(否则人真跑了会被冻结)。
            #       例外: 方位跳但距离基本不变 且 主相机换了 —— 这是真实的
            #       跨相机切换, 必须立刻接受, 否则切换机动性会变差。
            if self._fdist is not None and self._fbrg is not None:
                _jd = abs(vdist - self._fdist)
                _jb = abs(wrap(bearing - self._fbrg))
                _cam_chg = (vis_cam != self._fcam)
                _real_switch = (_jb > math.radians(35.0) and _jd < 0.28
                                and _cam_chg)
                if (_jd > 0.45 or _jb > math.radians(35.0)) \
                        and not _real_switch and now < self._fhold_until:
                    vdist, bearing = self._fdist, self._fbrg
                else:
                    if (_jd > 0.45 or _jb > math.radians(35.0)) \
                            and not _real_switch:
                        self._fhold_until = now + 0.60
                    a_f = 0.55
                    self._fdist = (1 - a_f) * self._fdist + a_f * vdist
                    self._fbrg = wrap((1 - a_f) * self._fbrg
                                      + a_f * bearing)
                    vdist, bearing = self._fdist, self._fbrg
            else:
                self._fdist, self._fbrg = vdist, bearing
            self._fcam = vis_cam
            px = rx + math.cos(ryaw + bearing) * vdist
            py = ry + math.sin(ryaw + bearing) * vdist
            self._last_vis_xy = (px, py)      # 只由视觉刷新""",
    'B1 滤波逻辑')

# ============ C: 把绕障块整体搬到 ALIGN 之前 ============
AVOID_START = '        # --- 4. 绕障(P46): 锁存式 Bug2 ---'
AVOID_END = """            _dn = ('绕障%s 前%.2f 侧%.2f 目标%.2f'
                   % ('左' if self._avoid_side > 0 else '右',
                      _df, _side_room, dist))
            note = _dn
        else:
            self._det_dir = 0.0"""
i0 = s.index(AVOID_START)
i1 = s.index(AVOID_END) + len(AVOID_END)
block = s[i0:i1]

# 从原位删除, 留一句说明
s = s[:i0] + """        # --- 4. 绕障已上移到 ALIGN 之前(P47) ---
        # 原来放在这里, 但 `if self._aligning:` 分支在它之前就 return 了,
        # 人站在障碍后面时 brg 跳进 ±50~120°, 直接进 ALIGN 原地转, 绕障
        # 永远执行不到。见"3.5 绕障抢占"。
""" + s[i1:]
n += 1
print('   ok C1 移除原位')

# 改造成抢占式: 去掉 note=... , 加 return
block = block.replace("""            note = _dn
        else:
            self._det_dir = 0.0""", """            v_t, w_t, _sn = self._safety_gate(v_t, w_t, now)
            self._publish_smooth(v_t, w_t, now)
            self._log(now, src, dist, pspd, rspd, w_t, brg, _dn, nscan,
                      rx, ry, px, py)
            return""")

# 头部注释改成"抢占"
block = block.replace('        # --- 4. 绕障(P46): 锁存式 Bug2 ---',
                      '        # --- 3.5 绕障抢占(P46/P47): 锁存式 Bug2 ---')

ANCHOR = """        v_t = 0.0
        w_t = 0.0
        note = ''

        if src == '扫描':"""
NEW_ANCHOR = """        v_t = 0.0
        w_t = 0.0
        note = ''

""" + block + """
        if _avoid_now:
            # 绕障接管: 必须绕过 ALIGN。否则 brg 一跳出 band 就进"原地
            # 对正"并 return, 机器人只会原地转(实测 30s 挪 0.23m,
            # w 在 ±1.5 之间每 1s 翻号)。
            self._aligning = False

        if src == '扫描':"""
sub(ANCHOR, NEW_ANCHOR, 'C2 插入抢占块')

io.open(SRC, 'w', encoding='utf-8', newline='\n').write(s)
print('P47 应用完成 %d 处' % n)
import ast
ast.parse(io.open(SRC, encoding='utf-8').read())
print('语法 OK')
