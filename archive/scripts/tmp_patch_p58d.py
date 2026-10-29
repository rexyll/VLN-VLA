#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""P58d: 挂载规划器 + 安全距硬闸(需求4) + 与人保持距离(需求5/6)。"""
import io

P = '_p58_follower.py'
s = io.open(P, encoding='utf-8').read()

# ---------------- A. __init__ 状态 ----------------
OLD_A = """        self._vfh_hint = ''           # 日志用"""
assert s.count(OLD_A) == 1
s = s.replace(OLD_A, OLD_A + """
        # P58: 局部路径规划(需求2/3) + 视野基元(需求1)
        self._planner = LocalPlanner(self.cfg)
        self._prim_prev = ''          # 上一帧的基元, 只在切换时打日志
        self._plan_mode_on = False    # 是否正在按规划路径走
        self._plan_dbg = ''           # 当前规划摘要(供 status)""", 1)

# ---------------- B. 规划调度方法 ----------------
OLD_B = "    def _tick(self):\n"
assert s.count(OLD_B) == 1
NEW_B = '''    # ---------- 局部规划(P58) ----------
    def _maybe_replan(self, now, rx, ry, gx, gy):
        """按需触发重规划。目标先推离膨胀区, 否则"人贴障碍站着"时
        终点格永远占据, A* 只能退到"最近可达格", 会绕到障碍另一侧。"""
        _pl = self._planner
        if _pl.busy:
            return
        tgx, tgy = self._free_target(
            gx, gy, float(self.cfg['obstacle_clear_m']) + 0.06)
        if not _pl.need_replan(now, rx, ry, tgx, tgy):
            return
        _pl.busy = True
        th = threading.Thread(target=self._plan_worker,
                              args=(rx, ry, tgx, tgy, now), daemon=True)
        th.start()

    def _plan_worker(self, rx, ry, gx, gy, now):
        try:
            self._planner.plan_now(rx, ry, gx, gy, now)
        except Exception as e:                      # noqa: BLE001
            self.get_logger().warn('FOLLOWER 规划异常: %s' % e)
        finally:
            self._planner.busy = False

    def _tick(self):
'''
s = s.replace(OLD_B, NEW_B, 1)

# ---------------- C. 安全距硬闸(需求4) ----------------
OLD_C = """        stop_d = self.cfg['lidar_stop_m']
        slow_d = self.cfg['lidar_slow_m']"""
assert s.count(OLD_C) == 1
NEW_C = """        # P58 需求4: 与障碍**表面**的最小间隙(默认 0.20m, 配置文件可改)。
        # 口径与评测脚本的 min_gap 完全一致: geo_gap(车心) - ROBOT_R。
        # 只拦"继续前进会让间隙更小且将低于阈值"的动作, 不搞"间隙小于阈
        # 值就禁行" —— 后者会把沿墙直行、刚被挤进临界区但正在离开的机器
        # 人全部停死, P50 那类回归就是这么来的。
        _clr = float(self.cfg['obstacle_clear_m'])
        _gmin, _gobj = 99.0, None
        for _g in GEO:
            _gg = geo_gap(self.rx, self.ry, _g)
            if _gg < _gmin:
                _gmin, _gobj = _gg, _g
        _gap_now = _gmin - ROBOT_R
        if v_t > 0.02 and _gap_now < _clr + 0.14 and _gobj is not None:
            _look = max(0.20, min(0.45, 0.26 + 0.30 * v_t))
            _nx = self.rx + math.cos(self.ryaw) * _look
            _ny = self.ry + math.sin(self.ryaw) * _look
            _gmin2 = 99.0
            for _g in GEO:
                _gg2 = geo_gap(_nx, _ny, _g)
                if _gg2 < _gmin2:
                    _gmin2 = _gg2
            if _gmin2 < _gmin - 1e-4 and (_gmin2 - ROBOT_R) < _clr:
                v_t = 0.0
                _sd = (-(_gobj[1] - self.rx) * math.sin(self.ryaw)
                       + (_gobj[2] - self.ry) * math.cos(self.ryaw))
                if abs(w_t) < 0.30:
                    w_t = -1.0 if _sd > 0 else 1.0
                note = (note + ' 安全距%.2f<%.2f' % (_gap_now, _clr)).strip()

        stop_d = self.cfg['lidar_stop_m']
        slow_d = self.cfg['lidar_slow_m']"""
s = s.replace(OLD_C, NEW_C, 1)

# ---------------- D. 与人保持距离(需求5/6) ----------------
OLD_D = """        if src == '搜索':
            err_d = dist - 0.45
            guard_d = 0.30
        else:
            err_d = dist - self.target_d
            # 安全距(低于此值必须后退), 下面限速也要用
            guard_d = max(0.55, self.target_d - tok - 0.25)"""
assert s.count(OLD_D) == 1
NEW_D = """        if src == '搜索':
            err_d = dist - 0.45
            guard_d = 0.30
            _keep = 0.30
        else:
            err_d = dist - self.target_d
            # 安全距(低于此值必须后退), 下面限速也要用
            guard_d = max(0.55, self.target_d - tok - 0.25)
            # P58 需求5: 与目标的最小距离单独可配(默认 0.70m)。
            # 它比 guard_d 更严时以它为界 —— "必须保持在设定值之上"。
            _keep = max(guard_d, float(self.cfg['person_keep_m']))"""
s = s.replace(OLD_D, NEW_D, 1)

OLD_D2 = """        v_lat = v_ff + room / lat
        v_cap = max(0.0, min(0.72, v_lat))"""
assert s.count(OLD_D2) == 1
NEW_D2 = """        v_lat = v_ff + room / lat
        v_cap = max(0.0, min(0.72, v_lat))
        # P58 需求6: 接近目标必须减速。按"离设定距离还剩多少"给上限,
        # 0.55s 内刚好走完剩余量 —— 与视觉时延同量级, 不会冲过头。
        if dist < float(self.cfg['person_slow_m']) and _keep > 0.0:
            _v_app = max(0.10, (dist - _keep) / 0.55)
            if v_cap > _v_app:
                v_cap = _v_app"""
s = s.replace(OLD_D2, NEW_D2, 1)

OLD_D3 = """        # --- 8. 防撞人 ---
        if dist < guard_d:"""
assert s.count(OLD_D3) == 1
NEW_D3 = """        # --- 8. 防撞人(需求5: 人朝机器人走来时必须退让) ---
        if dist < _keep:"""
s = s.replace(OLD_D3, NEW_D3, 1)

OLD_D4 = """            if back_clear:
                v_t = max(min(v_t, -(0.2 + 0.7 * (guard_d - dist))), -0.5)"""
assert s.count(OLD_D4) == 1
NEW_D4 = """            if back_clear:
                v_t = max(min(v_t, -(0.2 + 0.7 * (_keep - dist))), -0.55)"""
s = s.replace(OLD_D4, NEW_D4, 1)

# ---------------- E. status 增加规划摘要 ----------------
OLD_E = """              'note': getattr(self, '_last_note', '')}"""
assert s.count(OLD_E) == 1
NEW_E = """              'note': getattr(self, '_last_note', ''),
              'prim': self._prim_prev,
              'plan': {'n': len(self._planner.pts),
                       'ver': self._planner.version,
                       'lv': round(float(self._planner.level), 3),
                       'reached': bool(self._planner.reached),
                       'on': bool(self._plan_mode_on),
                       'pts': [[round(x, 3), round(y, 3)]
                               for (x, y) in self._planner.pts[:40]]}}"""
s = s.replace(OLD_E, NEW_E, 1)

io.open(P, 'w', encoding='utf-8', newline='\n').write(s)
print('d ok')
