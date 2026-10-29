#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tmp_patch_p32.py —— P32: 严格视觉模式 + 丢失处理重写 + GOTO 到点修复

改动(自文件尾向头依次替换, 避免行号漂移):
  R1  _publish 追加 /follower/status 状态发布
  R2  _log 的模式标签识别 搜索/丢失
  R3  TRACK 段: GOTO 态用独立 target_d/guard_d, 能真正走到目标点
  R4  第2段: GOTO/SCAN 状态机重写(轮次上限 + 到点判定 + 超时)
  R5  第1段: 严格视觉模式(禁真值兜底) + 丢失触发
  R6  新增 _enter_scan / _infer_target 重写(用满最后 10 张快照)
  R7  _vis_cb 缓存最后一帧"检出"
  R8  __init__ 新增状态与 status 发布器
  R9  DEFAULT_CFG 新增 5 个配置项
"""
import io
import sys

P = 'vla_ws/src/vla_follow/vla_follow/follower.py'
src = io.open(P, encoding='utf-8').read()
if 'P32' in src:
    print('!! 已打过 P32 补丁, 退出')
    sys.exit(1)
lines = src.split('\n')


def idx(marker, tag, start=0):
    for i in range(start, len(lines)):
        if lines[i] == marker:
            return i
    print('!! 找不到锚点 [%s]: %r' % (tag, marker))
    sys.exit(1)


def splice(start_marker, end_marker, new_text, tag):
    """把 [start_marker, end_marker) 之间的行替换为 new_text"""
    i = idx(start_marker, tag)
    j = idx(end_marker, tag + '.end', i + 1)
    old_n = j - i
    new = new_text.split('\n')
    lines[i:j] = new
    print('   ok %-10s 替换 %3d 行 → %3d 行' % (tag, old_n, len(new)))


# ============ R1: _publish + _publish_status ============
R1 = '''    def _publish(self, v, w):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self.pub.publish(msg)
        self._publish_status()

    def _publish_status(self):
        """P32: 对外状态快照(5Hz 限流), 供评测脚本/上位机消费"""
        now = time.monotonic()
        if now - self._pub_status_t < 0.2:
            return
        self._pub_status_t = now
        vv = self.vis or {}
        det = bool(vv.get('detected'))
        st = {'track': self._track, 'src': self._src_prev,
              'lost': bool(self._lost),
              'lost_s': round(now - self._lost_at, 2) if self._lost else 0.0,
              'scan_round': int(self._scan_round),
              'detected': det,
              'cam': str(vv.get('cam', '')) if det else '',
              'vis_dist': (round(float(vv.get('dist', -1.0)), 3)
                           if det else -1.0),
              'bearing_deg': (round(math.degrees(float(vv.get('bearing', 0.0))), 1)
                              if det else None),
              'goto': ([round(float(self._search_target[0]), 2),
                        round(float(self._search_target[1]), 2)]
                       if (self._track == 'GOTO' and self._search_target)
                       else None),
              'cmd_v': round(float(self.v), 3),
              'cmd_w': round(float(self._last_w), 3)}
        m = String()
        m.data = json.dumps(st)
        self._status_pub.publish(m)

'''
splice('    def _publish(self, v, w):', 'def main(args=None):', R1, 'R1')

# ============ R2: _log 模式标签 ============
R2 = '''            if self._aligning:
                mode = '对正'
            elif src == '扫描':
                mode = '扫描'
            elif src == '搜索':
                mode = '前往'
            elif src == '丢失':
                mode = '丢失'
            else:
                mode = '跟踪'
            self.get_logger().info(
'''
splice("            mode = '对正' if self._aligning else ('扫描' if src == '扫描'",
       "                f'FOLLOWER [{mode}|{src}] 距离={dist:.2f}m'",
       R2, 'R2')

# ============ R3: TRACK 段的 GOTO 目标距 ============
R3 = '''        # ---- TRACK: 已对正 → 距离环决定前进/后退/保持 ----
        # P32: GOTO(前往人最后位置)态的目标是"到点", 不是"保持跟随距离"。
        # 若沿用 target_d=1.0 / guard_d=0.65, 机器人会在离目标点 1.0m 处就
        # 停住, 永远进不了 0.5m 的到点判定 —— 原实现里"先移动到人最后所处
        # 的位置"这一步其实走不完, 只能等超时。
        tok = self.cfg['track_dist_tol_m']
        if src == '搜索':
            err_d = dist - 0.45
            guard_d = 0.30
        else:
            err_d = dist - self.target_d
            # 安全距(低于此值必须后退), 下面限速也要用
            guard_d = max(0.55, self.target_d - tok - 0.25)
        tol = tok'''
splice('        # ---- TRACK: 已对正 → 距离环决定前进/后退/保持 ----',
       '        dtc = 0.05',
       R3, 'R3')

# ============ R4: 第2段 GOTO/SCAN 状态机 ============
R4 = '''        # --- 2. 搜索/扫描状态机 (P32) ---
        if self._track == 'GOTO':
            if now - self._search_t0 > self.cfg['goto_timeout_s']:
                self.get_logger().warn(
                    'FOLLOWER 前往最后位置超时(%.0fs), 转原地扫描'
                    % self.cfg['goto_timeout_s'])
                self._enter_scan(now, announce=False)
            else:
                gx, gy = self._search_target
                if math.hypot(gx - rx, gy - ry) < 0.5:
                    self.get_logger().info(
                        'FOLLOWER 已到达人最后出现位置(%+.1f,%+.1f), '
                        '原地 360° 扫描确认' % (gx, gy))
                    self._enter_scan(now, announce=False)
                else:
                    px, py = gx, gy
                    brg = wrap(math.atan2(gy - ry, gx - rx) - ryaw)
                    src = '搜索'
        elif self._track == 'SCAN':
            if now > self._scan_until:
                tgt = None
                desc = ''
                if self._scan_round < int(self.cfg['scan_rounds']) and self.hist:
                    tgt, desc = self._infer_target()
                if tgt is not None:
                    self._scan_round += 1
                    self._search_target = tgt
                    self._search_t0 = now
                    self._track = 'GOTO'
                    self.get_logger().info(
                        'FOLLOWER 扫描无果, 按快照再次推测 %s' % desc)
                else:
                    self._enter_scan(now, announce=True)
            src = '扫描'
'''
splice('        # --- 2. 搜索/扫描状态 ---', "            src = '扫描'", R4, 'R4')

# ============ R5: 第1段 目标估计 ============
R5 = '''        # --- 1. 目标估计 (P32: 严格视觉模式) ---
        # 只有"视觉确实检出"才允许驱动跟随。丢失后不再回落到 Gazebo 真值
        # 位姿 —— 那样机器人其实一直知道人在哪, 丢失处理等于没生效, 也无从
        # 验证"按最后 10 张抓拍推测位置"这条链路。allow_gt_track=True 仅供对照。
        vis = self._vis_ok
        lost_dt = (now - self._vis_mono) if self._vis_mono > 0 else 1e9
        vision_ok = (vis is not None and vis.get('dist', -1) > 0
                     and lost_dt <= self.cfg['lost_grace_s'])
        src = '真值'
        if vision_ok:
            vis_cam = str(vis.get('cam', 'front'))
            bearing = float(vis['bearing'])     # 相对机器人前向, + = 左
            vdist = float(vis['dist'])
            if self._track != 'TRACK':
                gap = (now - self._lost_at) if self._lost else 0.0
                self.get_logger().info(
                    'FOLLOWER ★ 重新捕获目标([%s]相机, 偏离中线%+.0f°, %.2fm)%s'
                    ' → 立即对正跟踪'
                    % (vis_cam,
                       math.degrees(float(vis.get('bearing_cam', 0.0))),
                       vdist,
                       (' 丢失%.1fs后恢复' % gap) if self._lost else ''))
                self._track = 'TRACK'
                self._lost = False
                self._lost_announced = False
                self._scan_round = 0
                self._reacq_until = now + 8.0
            px = rx + math.cos(ryaw + bearing) * vdist
            py = ry + math.sin(ryaw + bearing) * vdist
            self._last_vis_xy = (px, py)      # 只由视觉刷新
            if vis_cam == 'front':
                if self._vis_prev is not None:
                    dt = now - self._vis_prev[0]
                    if dt > 0.15:
                        dd = (self._vis_prev[1] - vdist) / dt
                        self.vis_pspd = 0.7 * self.vis_pspd + \\
                            0.3 * max(0.0, min(1.0, dd))
                        self._vis_prev = (now, vdist)
                else:
                    self._vis_prev = (now, vdist)
            else:
                self._vis_prev = None
            pspd = self.vis_pspd
            src = '视觉' if vis_cam == 'front' else f'视觉-{vis_cam}'
            brg = bearing                       # 相机直接测得的相对方位
        else:
            self._vis_prev = None
            self.vis_pspd = 0.0
            if self.cfg['allow_gt_track']:
                src = '真值'                    # 兼容: 旧的真值兜底行为
            else:
                src = '丢失'
                if self._last_vis_xy is not None:
                    px, py = self._last_vis_xy  # 停在最后确认位置
            brg = wrap(math.atan2(py - ry, px - rx) - ryaw)
            # 判定丢失 → 用最后 10 张抓拍推测人最后所处的位置
            if (self._track == 'TRACK' and not self.cfg['allow_gt_track']
                    and lost_dt > self.cfg['lost_grace_s']):
                if not self._lost:
                    self._lost = True
                    self._lost_at = now
                n_hist = min(len(self.hist), 10)
                if self._last_vis_xy is None or n_hist == 0:
                    self.get_logger().warn(
                        'FOLLOWER ✗ 目标丢失(无视觉历史/快照为空) → 原地扫描')
                    self._enter_scan(now, announce=False)
                elif self._scan_round < int(self.cfg['scan_rounds']):
                    tgt, desc = self._infer_target()
                    if tgt is not None:
                        self._scan_round += 1
                        self._search_target = tgt
                        self._search_t0 = now
                        self._track = 'GOTO'
                        self.get_logger().warn(
                            'FOLLOWER ✗ 目标丢失(%.1fs 无检出), 依据最后 %d 张'
                            '抓拍推测: %s → 前往该位置'
                            % (lost_dt, n_hist, desc))
                    else:
                        self._enter_scan(now, announce=False)
                else:
                    self._enter_scan(now, announce=True)
'''
splice('        # --- 1. 目标估计: 视觉优先, 真值兜底 ---',
       '        # --- 2. 搜索/扫描状态机 (P32) ---', R5, 'R5')

# ============ R6: _enter_scan + _infer_target ============
R6 = '''    def _enter_scan(self, now, announce=False):
        """P32: 进入原地 360° 扫描态; announce=True 时明确提示"目标丢失" """
        self._track = 'SCAN'
        self._scan_until = now + float(self.cfg['scan_dur_s'])
        if announce and not self._lost_announced:
            self._lost_announced = True
            where = ('(%+.1f,%+.1f)' % self._last_vis_xy
                     if self._last_vis_xy is not None else '(未知)')
            self.get_logger().error(
                'FOLLOWER ✗✗ 目标丢失: 已按最后 %d 张快照前往并搜索 %d 轮仍无'
                '检出, 最后确认位置%s → 原地扫描, 等待目标重新进入视野'
                % (min(len(self.hist), 10), self._scan_round, where))

    def _infer_target(self):
        """P32: 用最后 10 张快照推测人最后所处的位置。

        位置取窗口尾部 1/3 的均值(最近几帧最可信, 人可能已驻足),
        速度用窗口首末差(抑制单帧噪声), 外推 1.5s 且限幅 1.2m ——
        要求是"先移动到人最后所处的位置", 不宜激进外推。
        """
        CAM_CN = {'front': '前', 'left': '左', 'right': '右', 'back': '后'}
        if not self.hist:
            return None, '无快照历史'
        n = min(10, len(self.hist))
        win = self.hist[-n:]
        k = max(1, n // 3)
        tail = win[-k:]
        lx = sum(float(h['wx']) for h in tail) / len(tail)
        ly = sum(float(h['wy']) for h in tail) / len(tail)
        tx, ty = lx, ly
        spd = 0.0
        if n >= 2:
            dt = max(float(win[-1]['t']) - float(win[0]['t']), 1.0)
            vx = (float(win[-1]['wx']) - float(win[0]['wx'])) / dt
            vy = (float(win[-1]['wy']) - float(win[0]['wy'])) / dt
            spd = math.hypot(vx, vy)
            if spd > 0.12:                    # 站立不动时不外推
                ex, ey = vx * 1.5, vy * 1.5
                e = math.hypot(ex, ey)
                if e > 1.2:
                    ex, ey = ex * 1.2 / e, ey * 1.2 / e
                tx, ty = lx + ex, ly + ey
        tx = max(-5.3, min(5.3, tx))
        ty = max(-5.3, min(5.3, ty))
        seq = [h.get('cam', '?') for h in win]
        seq_cn = ','.join(CAM_CN.get(c, c) for c in seq[-6:])
        lateral = len(set(seq)) > 1
        desc = ('方位序列[%s]%s, 末位置(%+.1f,%+.1f) 人速%.2f → 前往(%+.1f,%+.1f)'
                % (seq_cn, '横向移动' if lateral else '同相机',
                   lx, ly, spd, tx, ty))
        return (tx, ty), desc

'''
splice('    def _infer_target(self):', '    # ---------- 绕障航点 ----------', R6, 'R6')

# ============ R7: _vis_cb 缓存最后检出帧 ============
R7 = '''    def _vis_cb(self, msg):
        """相机人检测: {detected, cam, dist, bearing, bearing_cam, u, area}"""
        try:
            v = json.loads(msg.data)
        except Exception:
            return
        self.vis = v
        self._vis_msg_mono = time.monotonic()
        # P23: 只有"确实检出"才刷新检出生效时刻。
        # 原实现每帧无条件刷新(包括 detected=false), 于是
        # "now - _vis_mono > 2.0"(视觉丢失 2s) 永远不成立 ——
        # 目标丢失后的 GOTO→SCAN 搜索链路整段是死代码, 实测 24 次
        # "四相机视野内均无人" 却 0 次触发搜索。
        # P32: 同时缓存最后一帧"检出"结果, 单帧漏检时上层还能在
        # grace 窗口内继续用这一帧跟随, 而不是立刻掉进丢失分支。
        if v.get('detected'):
            self._vis_mono = self._vis_msg_mono
            self._vis_ok = v

'''
splice('    def _vis_cb(self, msg):', '    def _hist_cb(self, msg):', R7, 'R7')

# ============ R8: __init__ 状态 ============
R8 = '''        self._reacq_until = 0.0
        # P32: 严格视觉模式 / 丢失处理状态
        self._vis_ok = None           # 最后一帧"确实检出"的视觉结果
        self._last_vis_xy = None      # 最后一次视觉确认的目标世界坐标
        self._lost = False            # 是否处于目标丢失态
        self._lost_at = 0.0           # 丢失开始时刻
        self._scan_round = 0          # "推测→前往→扫描"已完成轮次
        self._lost_announced = False  # 是否已提示"目标丢失"
        self._status_pub = self.create_publisher(
            String, '/follower/status', 10)
        self._pub_status_t = 0.0'''
splice('        self._reacq_until = 0.0', '        self.hist = []', R8, 'R8')

# ============ R9: DEFAULT_CFG ============
R9 = """    'blk_s': 2.20,                  # P27 原地受阻多久后改后退脱困
    # P32: 丢失处理(严格视觉模式) —— 视觉丢失后不再用 Gazebo 真值跟踪
    'allow_gt_track': False,        # 是否允许真值兜底(仅供对照, 默认关)
    'lost_grace_s': 0.80,           # 连续多久无检出才判定"目标丢失"
    'scan_dur_s': 12.0,             # 到达最后位置后原地扫描时长
    'goto_timeout_s': 20.0,         # 前往最后位置的最长时间
    'scan_rounds': 2,               # 最多"推测→前往→扫描"轮次"""
splice("    'blk_s': 2.20,                  # P27 原地受阻多久后改后退脱困",
       '}', R9, 'R9')

out = '\n'.join(lines)
io.open(P, 'w', encoding='utf-8', newline='\n').write(out)
print('P32_APPLIED  %d → %d 行' % (len(src.split('\n')), len(lines)))
