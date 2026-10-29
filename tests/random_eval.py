#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tmp_random_eval.py —— "人在场景内随机走动"综合跟随评测

覆盖的需求:
  1. 跟随距离稳定在 1.0 ± 0.1 m
  2. 主动避开障碍物, 不发生碰撞
  3. 人跨相机视野时机器人快速调整姿态(恢复 front 对正)
  4. 人从所有相机消失 → 按最后 10 张快照推测位置并前往 → 到点扫描
     → 仍无检出则提示"目标丢失" → 目标重新出现后立即对正跟踪

用法:
  python3 tests/random_eval.py --dur 180 --out eval_random
"""
import argparse
import json
import math
import os
import subprocess
import sys
import time

# 项目根: 所有依赖与产物都从这里解析, 不再硬编码
VLA_HOME = os.environ.get('VLA_HOME', '/home/earl/workspace/VLA')

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import gz.transport13 as gz_transport
from gz.msgs10.pose_v_pb2 import Pose_V as GzPose_V

ROBOT_R = 0.18          # 车体外接半径
WORLD = 'follow_world'

# P79: 距离合格区间改为**从 vision_config.json 读**(track_dist_m ± tol)。
# 原先硬编码 0.9~1.1, 用户把跟随区间放宽到 0.5~1.5 后口径必须跟着变,
# 否则 in_band 会永远算出 0(全部记成 near)。
BAND_LO, BAND_HI = 0.5, 1.5


def load_band(cfg_path='vla_demo/vision_config.json'):
    global BAND_LO, BAND_HI
    try:
        with open(cfg_path, encoding='utf-8') as f:
            c = json.load(f)
        tgt = float(c.get('track_dist_m', 1.0))
        tol = float(c.get('track_dist_tol_m', 0.5))
        BAND_LO, BAND_HI = round(tgt - tol, 3), round(tgt + tol, 3)
        print('[band] 距离合格区间 = %.2f ~ %.2f m (来自 %s)' % (BAND_LO, BAND_HI, cfg_path))
    except Exception as e:
        print('[band] 读取配置失败, 用默认 %.2f~%.2f: %s' % (BAND_LO, BAND_HI, e))
    return BAND_LO, BAND_HI

# 与 work_area.sdf 严格一致的障碍(绝对坐标)
# kind, x, y, 半长a, 半宽b, yaw   (圆柱 b 不用)
OBS = [
    ('box', 3.5, 2.0, 0.40, 0.40, 0.0),
    ('box', -3.0, 3.2, 0.60, 0.30, 0.5),
    ('cyl', 0.5, -3.2, 0.40, 0.0, 0.0),
    ('cyl', -4.2, -2.5, 0.50, 0.0, 0.0),
    ('box', 4.2, -3.0, 0.30, 0.30, 0.0),
    ('box', -1.5, -4.5, 0.50, 0.25, -0.4),
    ('cyl', 2.2, 4.6, 0.35, 0.0, 0.0),
]


def obj_dist(x, y, ob):
    """点到障碍表面的距离(负数=在内部)"""
    kind, ox, oy, a, b, yaw = ob
    dx, dy = x - ox, y - oy
    if kind == 'cyl':
        return math.hypot(dx, dy) - a
    c, s = math.cos(-yaw), math.sin(-yaw)
    lx, ly = dx * c - dy * s, dx * s + dy * c
    qx, qy = abs(lx) - a, abs(ly) - b
    return math.hypot(max(qx, 0.0), max(qy, 0.0)) + min(max(qx, qy), 0.0)


def gap_obs(x, y):
    """到最近障碍表面的间隙(已扣车体外接半径)"""
    return min(obj_dist(x, y, ob) for ob in OBS) - ROBOT_R


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def set_pose(name, x, y, z, yaw=0.0):
    req = ('name: "%s", position: {x: %s, y: %s, z: %s}, '
           'orientation: {z: %s, w: %s}'
           % (name, x, y, z, math.sin(yaw / 2), math.cos(yaw / 2)))
    for _ in range(3):
        r = subprocess.run(
            ['gz', 'service', '-s', '/world/%s/set_pose' % WORLD,
             '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
             '--timeout', '3000', '--req', req],
            capture_output=True, text=True)
        if 'true' in (r.stdout or '').lower():
            return True
        time.sleep(0.3)
    return False


class Eval(Node):

    def __init__(self, dur):
        super().__init__('random_eval')
        self.dur = dur
        self.t0 = time.monotonic()
        self.done = False
        self.recs = []
        self.rx = self.ry = self.ryaw = None
        self.px = self.py = None
        self.vis = {}
        self.st = {}
        self.dg = {}
        self._last_cam = ''
        self._prev_lost = False
        self._prev_track = ''
        self.pub_cmd = self.create_publisher(String, '/robot/command', 10)
        # P37: 必须用 /world/*/dynamic_pose/info 的**世界坐标**。
        # /robot/odom 与 /person/odom 是差速插件的 odom 系(自启动累计里程),
        # set_pose 瞬移不会重置它们 —— 实测把 robot 设到 (0,0), odom 仍报
        # (0.50,-3.14), 用它算"人机距离"整轮指标都是错的。
        self._gz = gz_transport.Node()
        self._gz.subscribe(GzPose_V,
                           '/world/%s/dynamic_pose/info' % WORLD,
                           self._gt_cb)
        self.create_subscription(String, '/perception/person', self._vb, 20)
        self.create_subscription(String, '/follower/status', self._sb, 20)
        # P80: 同时记录 /follower/diag(10Hz) 的决策文字与安全层内部量。
        # 没有它, 评测只能看到"距离/角度"这类结果量, 无法回答
        # "为什么在 0.65m 还在往前给 +0.23" —— 只能靠事后猜机理。
        self.create_subscription(String, '/follower/diag', self._db, 20)
        self.create_timer(0.05, self._tick)
        self._last_prog = 0.0

    def _gt_cb(self, msg):
        for p in msg.pose:
            if p.name == 'robot':
                self.rx = p.position.x
                self.ry = p.position.y
                self.ryaw = yaw_of(p.orientation)
            elif p.name == 'person':
                self.px = p.position.x
                self.py = p.position.y

    def _vb(self, m):
        try:
            self.vis = json.loads(m.data)
        except Exception:
            pass

    def _sb(self, m):
        try:
            self.st = json.loads(m.data)
        except Exception:
            pass

    def _db(self, m):
        try:
            self.dg = json.loads(m.data)
        except Exception:
            pass

    def _tick(self):
        t = time.monotonic() - self.t0
        if t > self.dur:
            self.done = True
            return
        if None in (self.rx, self.ry, self.px, self.py):
            return
        v = self.vis or {}
        st = self.st or {}
        det = bool(v.get('detected'))
        cam = str(v.get('cam', '')) if det else ''
        brg = (math.degrees(float(v.get('bearing', 0.0))) if det else None)
        d = math.hypot(self.px - self.rx, self.py - self.ry)
        self.recs.append({
            't': round(t, 3), 'rx': self.rx, 'ry': self.ry,
            'ryaw': self.ryaw, 'px': self.px, 'py': self.py,
            'd': d, 'cam': cam, 'brg': brg,
            'det': det, 'vis_d': float(v.get('dist', -1.0)) if det else -1.0,
            'track': str(st.get('track', '')), 'src': str(st.get('src', '')),
            'lost': bool(st.get('lost', False)),
            'goto': st.get('goto'), 'cmd_v': float(st.get('cmd_v', 0.0)),
            'gap': gap_obs(self.rx, self.ry),
            # P80: 决策级字段(来自 /follower/diag)
            'note': str(self.dg.get('note', ''))[:40],
            'mode': str(self.dg.get('mode', '')),
            'td': self.dg.get('td'),
            'sl': self.dg.get('sl'),
            'dh': self.dg.get('dh'),
            'dl': self.dg.get('dl'),
            'dr': self.dg.get('dr'),
            'dw': self.dg.get('w'),
        })
        if t - self._last_prog > 15.0:
            self._last_prog = t
            print('[EVAL %5.1fs] d=%.2f cam=%-5s trk=%-5s lost=%d gap=%+.2f'
                  % (t, d, cam or '-', str(st.get('track', '') or '-'),
                     1 if st.get('lost') else 0, gap_obs(self.rx, self.ry)),
                  flush=True)


def response_time(t_sw, recs):
    """切换后恢复到 front 且 |brg|<=15° 且稳定 0.3s 的耗时"""
    for i, r in enumerate(recs):
        if r['t'] < t_sw:
            continue
        if r['cam'] == 'front' and r['brg'] is not None and abs(r['brg']) <= 15.0:
            win = recs[i:i + 6]
            if len(win) < 6:
                return None
            if all(w['cam'] == 'front' and w['brg'] is not None
                   and abs(w['brg']) <= 15.0 for w in win):
                return r['t'] - t_sw
    return None


def analyze(recs):
    out = {}
    if not recs:
        return out
    # ---------- 1. 距离 ----------
    # 只在"正常视觉跟踪"时段统计(排除开局追赶、前往/扫描/丢失段)
    live = [r for r in recs
            if r['t'] > 12.0 and r['src'].startswith('视觉') and not r['lost']]
    ds = [r['d'] for r in live]
    alld = [r['d'] for r in recs if r['t'] > 12.0]
    if ds:
        ds_sorted = sorted(ds)
        n = len(ds_sorted)
        out['dist'] = {
            'n': n,
            'mean': round(sum(ds) / n, 3),
            'median': round(ds_sorted[n // 2], 3),
            'std': round(math.sqrt(sum((x - sum(ds) / n) ** 2 for x in ds) / n), 3),
            'min': round(ds_sorted[0], 3),
            'max': round(ds_sorted[-1], 3),
            'band': [BAND_LO, BAND_HI],
            'in_band_pct': round(100.0 * sum(1 for x in ds if BAND_LO <= x <= BAND_HI) / n, 1),
            'near_pct': round(100.0 * sum(1 for x in ds if x < BAND_LO) / n, 1),
            'far_pct': round(100.0 * sum(1 for x in ds if x > BAND_HI) / n, 1),
            'min_all': round(min(alld), 3) if alld else None,
        }
    # 连续在带内统计
    runs, cur = [], 0
    for r in recs:
        if r['t'] <= 12.0:
            continue
        if r['src'].startswith('视觉') and not r['lost'] and BAND_LO <= r['d'] <= BAND_HI:
            cur += 1
        else:
            if cur:
                runs.append(cur * 0.05)
            cur = 0
    if cur:
        runs.append(cur * 0.05)
    if 'dist' not in out:
        out['dist'] = {}
    out['dist']['max_streak_s'] = round(max(runs), 2) if runs else 0.0

    # ---------- 1a. 贴身/接触统计(P80) ----------
    # 人裙摆半径 0.26 + 车体外接 0.18 = 中心距 0.44m 即"物理接触"。
    # 这一项必须**单独报**: 逐帧核查发现它几乎全部落在"检测丢失"窗口里
    # (视觉没了 → 距离环没有输入 → 人自己走到车身上), 与 in_band 反映的
    # 是完全不同的失效模式。把它混在 min_all 里, 会被误读成"距离环没调好"。
    CONTACT_D = 0.44
    evs_c, cur_c = [], None
    for r in recs:
        if r['t'] <= 12.0:
            continue
        hit = 0.0 < r['d'] < CONTACT_D
        if hit and cur_c is None:
            cur_c = {'t0': r['t'], 'n': 0, 'det': 0, 'dmin': r['d'],
                     'modes': set(), 'notes': set()}
        if cur_c is not None:
            if hit:
                cur_c['n'] += 1
                cur_c['det'] += 1 if r['det'] else 0
                cur_c['dmin'] = min(cur_c['dmin'], r['d'])
                if r['mode']:
                    cur_c['modes'].add(r['mode'])
                if r['note']:
                    cur_c['notes'].add(r['note'][:14])
            else:
                evs_c.append(cur_c)
                cur_c = None
    if cur_c is not None:
        evs_c.append(cur_c)
    out['contact'] = {
        'thr_m': CONTACT_D,
        'n': sum(e['n'] for e in evs_c),
        'events': len(evs_c),
        'det_pct': round(100.0 * sum(e['det'] for e in evs_c) /
                         max(1, sum(e['n'] for e in evs_c)), 1),
        'dmin': round(min((e['dmin'] for e in evs_c), default=0.0), 3),
        'detail': [{'t0': round(e['t0'], 1), 'dur': round(e['n'] * 0.05, 2),
                    'det': e['det'], 'dmin': round(e['dmin'], 3),
                    'mode': '/'.join(sorted(e['modes'])),
                    'note': ' | '.join(sorted(e['notes'])[:3])}
                   for e in evs_c],
    }

    # ---------- 1b. 居中率(用户主目标: 人在 front 视野中间, 左右摆动 ≤10°) ----------
    bs = sorted(abs(r['brg']) for r in live
                if r.get('cam') == 'front' and r.get('brg') is not None)
    if bs:
        m = len(bs)
        out['center'] = {
            'n': m,
            'within10_pct': round(100.0 * sum(1 for x in bs if x <= 10.0) / m, 1),
            'within15_pct': round(100.0 * sum(1 for x in bs if x <= 15.0) / m, 1),
            'median_deg': round(bs[m // 2], 1),
            'p90_deg': round(bs[min(m - 1, int(m * 0.9))], 1),
            'max_deg': round(bs[-1], 1),
        }

    # ---------- 2. 避障 ----------
    gaps = [r['gap'] for r in recs if r['t'] > 2.0]
    out['avoid'] = {
        'min_gap': round(min(gaps), 3) if gaps else None,
        'hit_n': sum(1 for g in gaps if g < 0.0),
        'brush_n': sum(1 for g in gaps if 0.0 <= g < 0.05),
    }

    # ---------- 3. 视野切换 ----------
    switches = []
    last = ''
    for r in recs:
        c = r['cam']
        if c and c != last:
            if last:
                switches.append((r['t'], last, c))
            last = c
    resp = []
    for (ts, frm, to) in switches:
        rt = response_time(ts, recs)
        if rt is not None:
            resp.append(rt)
    cross = [s for s in switches if abs(
        {'front': 0, 'left': 1, 'back': 2, 'right': 3}.get(s[2], 0)
        - {'front': 0, 'left': 1, 'back': 2, 'right': 3}.get(s[1], 0)) == 2]
    cross_resp = [response_time(s[0], recs) for s in cross]
    cross_resp = [x for x in cross_resp if x is not None]
    out['switch'] = {
        'n': len(switches),
        'resp_mean': round(sum(resp) / len(resp), 2) if resp else None,
        'resp_max': round(max(resp), 2) if resp else None,
        'resp_none': len(switches) - len(resp),
        'cross_n': len(cross),
        'cross_mean': round(sum(cross_resp) / len(cross_resp), 2) if cross_resp else None,
    }

    # ---------- 4. 丢失处理 ----------
    evs, cur = [], None
    prev_lost = False
    prev_track = ''
    goto_n = 0
    for r in recs:
        if r['lost'] and not prev_lost:
            cur = {'t0': r['t'], 't1': None, 'goto': 0, 'reached': 0,
                   'regen': None, 'goto_pt': None}
        if cur is not None:
            if r['track'] == 'GOTO' and prev_track != 'GOTO':
                cur['goto'] += 1
                cur['goto_pt'] = r['goto']
            if prev_track == 'GOTO' and r['track'] == 'SCAN':
                cur['reached'] += 1
            if prev_lost and not r['lost']:
                cur['t1'] = r['t']
                cur['regen'] = r['t'] - cur['t0']
                evs.append(cur)
                cur = None
        prev_lost = r['lost']
        prev_track = r['track']
    if cur is not None:
        cur['t1'] = recs[-1]['t']
        evs.append(cur)
    out['lost'] = {
        'n': len(evs),
        'goto_n': sum(e['goto'] for e in evs),
        'reached_n': sum(e['reached'] for e in evs),
        'dur_mean': round(sum(e['t1'] - e['t0'] for e in evs) / len(evs), 2) if evs else 0,
        'dur_max': round(max((e['t1'] - e['t0']) for e in evs), 2) if evs else 0,
        'regen_mean': round(sum(e['regen'] for e in evs if e['regen']) /
                            max(1, sum(1 for e in evs if e['regen'])), 2) if evs else 0,
        'events': [{'t0': round(e['t0'], 1), 'dur': round(e['t1'] - e['t0'], 1),
                    'goto': e['goto'], 'reached': e['reached'],
                    'goto_pt': e['goto_pt']} for e in evs],
    }

    # ---------- 5. 姿态机动性 ----------
    # 角速度指令绝对值均值/峰值(仅视觉跟踪段)
    ws = [abs(r['cmd_v']) for r in recs if r['t'] > 12]
    out['sanity'] = {
        'samples': len(recs),
        'vis_pct': round(100.0 * sum(1 for r in recs if r['det']) / len(recs), 1),
        'front_pct': round(100.0 * sum(1 for r in recs if r['cam'] == 'front') / len(recs), 1),
        'cmd_v_mean': round(sum(ws) / len(ws), 3) if ws else 0,
    }

    # ---------- 6. 后退频次(P79 用户口径: "避免频繁后退") ----------
    # 只看正常跟随段(与距离统计同一口径), 后退 = 指令线速度 < -0.03 m/s。
    seg = [r for r in recs if r['t'] > 12.0 and not r['lost']]
    if seg:
        BTH = -0.03
        n = len(seg)
        nback = sum(1 for r in seg if r['cmd_v'] < BTH)
        # 事件: 由"非后退"进入"后退"的次数 + 每次持续时长
        evs, in_ev, t0 = [], False, 0.0
        for r in seg:
            b = r['cmd_v'] < BTH
            if b and not in_ev:
                in_ev, t0 = True, r['t']
            elif not b and in_ev:
                in_ev = False
                evs.append(r['t'] - t0)
        if in_ev:
            evs.append(seg[-1]['t'] - t0)
        dur = max(1e-6, seg[-1]['t'] - seg[0]['t'])
        out['back'] = {
            'pct': round(100.0 * nback / n, 1),
            'events': len(evs),
            'per_100s': round(100.0 * len(evs) / dur, 1),
            'mean_dur_s': round(sum(evs) / len(evs), 2) if evs else 0.0,
            'max_dur_s': round(max(evs), 2) if evs else 0.0,
            'total_s': round(sum(evs), 1),
        }
    return out


def draw(recs, out_path):
    try:
        from PIL import Image, ImageDraw
    except Exception as e:
        print('!! 无 PIL, 跳过轨迹图: %s' % e)
        return None
    W = H = 700
    S = 55.0          # px / m, 世界 ±6.3m

    def P(x, y):
        return (W / 2 + x * S, H / 2 - y * S)

    im = Image.new('RGB', (W, H), (18, 20, 26))
    d = ImageDraw.Draw(im)
    # 场地 12x12
    d.rectangle([P(-6, 6)[0], P(-6, 6)[1], P(6, -6)[0], P(6, -6)[1]],
                outline=(70, 76, 92), width=2)
    for g in range(-5, 6):
        d.line([P(g, 6), P(g, -6)], fill=(34, 38, 48))
        d.line([P(-6, g), P(6, g)], fill=(34, 38, 48))
    # 障碍
    for ob in OBS:
        kind, ox, oy, a, b, yaw = ob
        if kind == 'cyl':
            d.ellipse([P(ox - a, oy + a), P(ox + a, oy - a)],
                      fill=(58, 64, 80), outline=(120, 130, 152))
        else:
            c, s = math.cos(yaw), math.sin(yaw)
            pts = []
            for (dx, dy) in ((-a, -b), (a, -b), (a, b), (-a, b)):
                pts.append(P(ox + dx * c - dy * s, oy + dx * s + dy * c))
            d.polygon(pts, fill=(58, 64, 80), outline=(120, 130, 152))
    # 人轨迹(蓝)
    if len(recs) > 1:
        d.line([P(r['px'], r['py']) for r in recs], fill=(56, 132, 255), width=2)
        # 机器人轨迹(红)
        d.line([P(r['rx'], r['ry']) for r in recs], fill=(232, 74, 74), width=2)
    # 丢失事件
    for r in recs:
        if r['lost'] and r['goto']:
            g = P(r['goto'][0], r['goto'][1])
            d.line([g[0] - 6, g[1] - 6, g[0] + 6, g[1] + 6], fill=(255, 190, 40), width=2)
            d.line([g[0] - 6, g[1] + 6, g[0] + 6, g[1] - 6], fill=(255, 190, 40), width=2)
            break
    # 起终点
    if recs:
        a = P(recs[0]['rx'], recs[0]['ry'])
        d.ellipse([a[0] - 5, a[1] - 5, a[0] + 5, a[1] + 5], fill=(120, 230, 150))
        b = P(recs[-1]['rx'], recs[-1]['ry'])
        d.ellipse([b[0] - 6, b[1] - 6, b[0] + 6, b[1] + 6], outline=(255, 255, 255), width=2)
    d.text((14, 12), 'blue=person  red=robot  green=start  X=goto target',
           fill=(190, 196, 210))
    im.save(out_path)
    return out_path


def main():
    global BAND_LO, BAND_HI
    ap = argparse.ArgumentParser()
    ap.add_argument('--dur', type=float, default=180.0)
    ap.add_argument('--out', default='eval_random')
    ap.add_argument('--outdir', default=os.path.join(VLA_HOME, 'tests', 'out'),
                    help='产物目录(json/png), 默认 $VLA_HOME/tests/out')
    ap.add_argument('--no-reset', action='store_true')
    ap.add_argument('--settle', type=float, default=2.0,
                    help='复位后等多久再下发 FOLLOW')
    ap.add_argument('--band', nargs=2, type=float, default=None,
                    metavar=('LO', 'HI'),
                    help='距离合格区间, 默认从 vision_config.json 的 '
                         'track_dist_m±tol 读')
    a = ap.parse_args()
    if a.band:
        global BAND_LO, BAND_HI
        BAND_LO, BAND_HI = a.band
        print('[band] 命令行指定 %.2f ~ %.2f m' % (BAND_LO, BAND_HI))
    else:
        load_band(os.path.join(VLA_HOME, 'vla_demo', 'vision_config.json'))
    os.makedirs(a.outdir, exist_ok=True)

    if not a.no_reset:
        # 机器人放原点朝 +x, 人放正前方 1.2m —— 开局即是"理想的跟随状态",
        # 免得把开局的追赶过程算进指标
        ok1 = set_pose('robot', 0.0, 0.0, 0.15, 0.0)
        ok2 = set_pose('person', 1.2, 0.0, 0.02, math.pi)
        print('[EVAL] 复位 robot=%s person=%s' % (ok1, ok2), flush=True)

    rclpy.init()
    node = Eval(a.dur)
    print('[EVAL] 等待 %ds 稳定后下发 FOLLOW...' % a.settle, flush=True)
    t_wait = time.monotonic() + a.settle
    while time.monotonic() < t_wait:
        rclpy.spin_once(node, timeout_sec=0.1)
    m = String()
    m.data = json.dumps({'action': 'FOLLOW', 'distance': 1.0,
                         'text': '请跟随我，保持1米的距离'})
    # rclpy 的 publisher 在"刚创建就发"的情况下实测不可靠(订阅端没收到),
    # 这里额外用命令行发(独立进程, DDS 发现完整; 命令写在
    # tests/follow_cmd.sh 里, 避免多层引号转义出错)
    for k in range(3):
        r = subprocess.run(['bash', os.path.join(VLA_HOME, 'tests',
                                                 'follow_cmd.sh')],
                           capture_output=True, text=True, timeout=30)
        print('[EVAL] FOLLOW 第%d次 CLI 下发 rc=%d %s'
              % (k + 1, r.returncode, (r.stderr or '').strip()[:60]),
              flush=True)
        node.pub_cmd.publish(m)
        rclpy.spin_once(node, timeout_sec=0.1)
    node.t0 = time.monotonic()
    print('[EVAL] FOLLOW 已下发, 开始 %ds 评测' % int(a.dur), flush=True)

    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.05)

    recs = node.recs
    res = analyze(recs)
    res['dur'] = a.dur
    out_json = os.path.join(a.outdir, '%s.json' % a.out)
    with open(out_json, 'w', encoding='utf-8') as f:
        json.dump({'summary': res, 'recs': recs}, f, ensure_ascii=False)
    png = draw(recs, os.path.join(a.outdir, '%s_traj.png' % a.out))
    print('\n================ 评测结果 (%ds) ================' % int(a.dur))
    print(json.dumps(res, ensure_ascii=False, indent=1))
    print('轨迹图: %s' % png)
    print('原始数据: %s' % out_json)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
