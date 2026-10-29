"""P73c: 前馈改为"距离变化率 + 本机前向速度", 并加近距离硬下界"""
import paramiko

HOME = '/home/earl/workspace/VLA'
SRC = HOME + '/vla_ws/src/vla_follow/vla_follow/follower.py'
DEP = HOME + '/deploy/follower.py'

cli = paramiko.SSHClient()
cli.set_missing_host_key_policy(paramiko.AutoAddPolicy())
cli.connect('192.168.255.128', 22, 'earl', '111111',
            timeout=10, look_for_keys=False, allow_agent=False)
sftp = cli.open_sftp()

with sftp.file(SRC, 'r') as f:
    s = f.read().decode('utf-8')

EDITS = []

# A. 标量斜率拟合
EDITS.append((
    "def fit_vel(win):\n",
    "def fit_slope(win):\n"
    "    \"\"\"最小二乘拟合 (t, v) 窗口的斜率 dv/dt, 样本不足返回 None。\"\"\"\n"
    "    n = len(win)\n"
    "    if n < 4:\n"
    "        return None\n"
    "    st = sum(w[0] for w in win)\n"
    "    sv = sum(w[1] for w in win)\n"
    "    stt = sum(w[0] * w[0] for w in win)\n"
    "    stv = sum(w[0] * w[1] for w in win)\n"
    "    den = n * stt - st * st\n"
    "    if den <= 1e-9:\n"
    "        return None\n"
    "    return (n * stv - st * sv) / den\n"
    "\n"
    "\n"
    "def fit_vel(win):\n",
))

# B. __init__ 状态
EDITS.append((
    "        self._pv_win = []\n"
    "        self._pv_cam = ''\n",
    "        self._pv_win = []\n"
    "        self._pv_cam = ''\n"
    "        # P73: 距离变化率观测窗 [(t, 视觉距离)], 与本机前向速度合成人速\n"
    "        self._rr_win = []\n"
    "        self._pvvr = 0.0        # 估计出来的人速(沿机头径向, 远离为正)\n"
    "        self.robot_vfwd = 0.0   # 本机前向速度(有符号, 真值位姿差分)\n",
))

# C. _pose_cb: 有符号前向速度
EDITS.append((
    "            if self._r_prev is not None and stamp > self._r_prev[0] + 1e-3:\n"
    "                pt, prx, pry = self._r_prev\n"
    "                inst = math.hypot(robot.position.x - prx,\n"
    "                                  robot.position.y - pry) / (stamp - pt)\n"
    "                self.robot_speed = 0.6 * self.robot_speed + 0.4 * min(inst, 2.0)\n",

    "            if self._r_prev is not None and stamp > self._r_prev[0] + 1e-3:\n"
    "                pt, prx, pry = self._r_prev\n"
    "                dtp2 = stamp - pt\n"
    "                inst = math.hypot(robot.position.x - prx,\n"
    "                                  robot.position.y - pry) / dtp2\n"
    "                self.robot_speed = 0.6 * self.robot_speed + 0.4 * min(inst, 2.0)\n"
    "                # P73: 有符号前向速度 —— 人速前馈要用它把\"距离变化率\"\n"
    "                # 折算成人的绝对速度(v_person = v_robot + d(dist)/dt)。\n"
    "                _yf = yaw_of(robot.orientation)\n"
    "                _vf = ((robot.position.x - prx) * math.cos(_yf)\n"
    "                       + (robot.position.y - pry) * math.sin(_yf)) / dtp2\n"
    "                self.robot_vfwd = 0.7 * self.robot_vfwd + \\\n"
    "                    0.3 * max(-1.5, min(1.5, _vf))\n",
))

# D. 视觉分支: 距离变化率 → 人速
EDITS.append((
    "            self._pv_t = now\n",
    "            self._pv_t = now\n"
    "            # P73: 人速改用\"距离变化率 + 本机前向速度\"(标量观测)。\n"
    "            # 原来的 2D 位置差分(EWMA)在 1.5m 处受 1° 方位量化影响,\n"
    "            # 单轴噪声 ±0.26m/s; 人走到 2.5m 时涨到 ±0.4 —— 实测抓到\n"
    "            # \"人正快速接近(0.83→0.55m)而 ff=+0.50(全速推)\"的反号样本,\n"
    "            # 把机器人一路顶到 0.41m, 人体占满画幅 → 整帧判未检出 →\n"
    "            # 丢失 → 原地扫描(120s 里 8 次/共 33s)。\n"
    "            # 距离变化率只需标量, 且 v_person = v_robot + d(dist)/dt 严格\n"
    "            # 自洽: 人不动时机器人怎么动都算出 0, 人匀速走时算出其真实\n"
    "            # 径向速度。用原始 dist(不做 P47 抖动抑制, 否则 hold 期间斜率\n"
    "            # 恒为 0 又变成\"人速=0\")。\n"
    "            _rawd = float(vis['dist'])\n"
    "            if self._rr_win and now - self._rr_win[-1][0] > 0.50:\n"
    "                self._rr_win = []\n"
    "            self._rr_win.append((now, _rawd))\n"
    "            while len(self._rr_win) > 1 and now - self._rr_win[0][0] > 0.70:\n"
    "                self._rr_win.pop(0)\n"
    "            _ddt = fit_slope(self._rr_win)\n"
    "            if _ddt is not None and abs(_ddt) < 2.5:\n"
    "                self._pvvr = max(-0.60, min(0.85,\n"
    "                                   self.robot_vfwd + _ddt))\n",
))

# E. TRACK: 用标量估计替代 2D 投影
EDITS.append((
    "        _vfwd = self._pvx * math.cos(ryaw) + self._pvy * math.sin(ryaw)\n",

    "        # P73: 前馈源改为距离变化率观测(见视觉分支的说明)。\n"
    "        _vfwd = self._pvvr\n",
))

# F. 近距离硬下界
EDITS.append((
    "        v_lat = v_ff + room / lat\n"
    "        v_cap = max(0.0, min(0.72, v_lat))\n",

    "        v_lat = v_ff + room / lat\n"
    "        v_cap = max(0.0, min(0.72, v_lat))\n"
    "        # P73: 近距离硬下界。人体近距离会把画面占满(W_REAL 的 55% 容差\n"
    "        # 直接失效), 一旦挤进 0.8m 内就必然整帧判未检出 → 丢失 → 扫描。\n"
    "        # 前馈/距离环任何一处算错都不许越过这条线; 真要更近就由 8 段\n"
    "        # 防撞人(dist<guard_d)去发后退。\n"
    "        if dist < 0.82:\n"
    "            v_cap = 0.0\n",
))

# G. 诊断日志加上前馈源
EDITS.append((
    "                f' err={self._dbg.get(\"err\", 0.0):+.2f}')\n",
    "                f' err={self._dbg.get(\"err\", 0.0):+.2f}'\n"
    "                f' vr={self.robot_vfwd:+.2f} pv={self._pvvr:+.2f}')\n",
))

for i, (old, new) in enumerate(EDITS):
    if s.count(old) != 1:
        raise SystemExit('第 %d 处锚点命中 %d 次, 中止:\n%s'
                         % (i + 1, s.count(old), old[:120]))
    s = s.replace(old, new, 1)

with sftp.file(SRC, 'w') as f:
    f.write(s.encode('utf-8'))
with sftp.file(DEP, 'w') as f:
    f.write(s.encode('utf-8'))

print('已打补丁 %d 处' % len(EDITS))
i, o, e = cli.exec_command(
    'cd %s && python3 -m py_compile %s && echo PY_OK' % (HOME, SRC),
    timeout=60)
print(o.read().decode(), e.read().decode()[:500])
i, o, e = cli.exec_command('md5sum %s %s' % (SRC, DEP), timeout=30)
print(o.read().decode())
sftp.close()
cli.close()
