"""P73: 修 path_free 静默停车否决 + 人速估计改窗口最小二乘"""
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

# ---------- A. path_free 增加 s_tight 返回值 ----------
EDITS.append((
    "    best = 99.0\n"
    "    tight = True\n"
    "    s = step\n"
    "    while s <= s_max + 1e-9:\n"
    "        nx = x + math.cos(ya) * s\n"
    "        ny = y + math.sin(ya) * s\n"
    "        if abs(nx) > WALL_HARD or abs(ny) > WALL_HARD:\n"
    "            return max(0.0, s - step), tight, 'wall'\n"
    "        gmin, gobj = 99.0, None\n"
    "        for g in GEO:\n"
    "            gp = geo_gap(nx, ny, g)\n"
    "            if gp < gmin:\n"
    "                gmin, gobj = gp, g\n"
    "        if gmin < best:\n"
    "            best = gmin\n"
    "        if gmin <= ROBOT_R + margin:\n"
    "            tight = False\n"
    "        if gmin <= 0.02:            # 车心即将触到表面 → 这就是可用距离\n"
    "            return max(0.0, s - step), tight, gobj\n"
    "        s += step\n"
    "    return s_max, tight, None\n",

    "    best = 99.0\n"
    "    tight = True\n"
    "    s_tight = s_max        # P73: 窄段起点(第一次 tight 失效处的弧长)\n"
    "    s = step\n"
    "    while s <= s_max + 1e-9:\n"
    "        nx = x + math.cos(ya) * s\n"
    "        ny = y + math.sin(ya) * s\n"
    "        if abs(nx) > WALL_HARD or abs(ny) > WALL_HARD:\n"
    "            return max(0.0, s - step), tight, 'wall', s_tight\n"
    "        gmin, gobj = 99.0, None\n"
    "        for g in GEO:\n"
    "            gp = geo_gap(nx, ny, g)\n"
    "            if gp < gmin:\n"
    "                gmin, gobj = gp, g\n"
    "        if gmin < best:\n"
    "            best = gmin\n"
    "        if gmin <= ROBOT_R + margin:\n"
    "            if tight:\n"
    "                s_tight = s\n"
    "            tight = False\n"
    "        if gmin <= 0.02:            # 车心即将触到表面 → 这就是可用距离\n"
    "            return max(0.0, s - step), tight, gobj, s_tight\n"
    "        s += step\n"
    "    return s_max, tight, None, s_tight\n",
))

# ---------- B. 人速估计: 模块级最小二乘 ----------
EDITS.append((
    "def wrap(a):\n",
    "def fit_vel(win):\n"
    "    \"\"\"最小二乘拟合窗口内 (t, x, y) 的速度 → (vx, vy), 样本不足返回 None。\n"
    "\n"
    "    P73。原来是相邻帧差分的 EWMA(a=0.35): 视觉方位量化约 1°, 在 1.5m\n"
    "    处相当于 2.6cm, 10Hz 下就是 0.26m/s 的单轴噪声, EWMA 之后仍有\n"
    "    ±0.12; 人走到 2.5m 时噪声涨到 ±0.2 —— 前馈于是变成\"随机推/随机\n"
    "    拖\"。实测日志抓到目标正在远离(距离 1.44→1.56 单调增)而\n"
    "    ff=-0.25(全速倒拖)的样本。窗口拟合噪声按 1/sqrt(n) 收, 且没有\n"
    "    EWMA 的相位滞后。\n"
    "    \"\"\"\n"
    "    n = len(win)\n"
    "    if n < 4:\n"
    "        return None\n"
    "    st = sum(w[0] for w in win)\n"
    "    stt = sum(w[0] * w[0] for w in win)\n"
    "    den = n * stt - st * st\n"
    "    if den <= 1e-9:\n"
    "        return None\n"
    "    sx = sum(w[1] for w in win)\n"
    "    sy = sum(w[2] for w in win)\n"
    "    stx = sum(w[0] * w[1] for w in win)\n"
    "    sty = sum(w[0] * w[2] for w in win)\n"
    "    return ((n * stx - st * sx) / den, (n * sty - st * sy) / den)\n"
    "\n"
    "\n"
    "def wrap(a):\n",
))

# ---------- C. __init__: 样本窗 ----------
EDITS.append((
    "        self._pvx = 0.0\n"
    "        self._pvy = 0.0\n",
    "        self._pvx = 0.0\n"
    "        self._pvy = 0.0\n"
    "        # P73: 视觉世界坐标样本窗[(t, px, py)], 供 fit_vel 估人速\n"
    "        self._pv_win = []\n"
    "        self._pv_cam = ''\n",
))

# ---------- D. 替换 EWMA 估速 ----------
EDITS.append((
    "            _v_inst = None\n"
    "            if self._pv_t is not None:\n"
    "                dtp = now - self._pv_t\n"
    "                if 0.09 < dtp < 0.60:\n"
    "                    _v_inst = math.hypot(px - self._pv_x,\n"
    "                                         py - self._pv_y) / dtp\n"
    "                    if _v_inst <= 2.0:        # 剔除切换跳变造成的假峰\n"
    "                        a = 0.35\n"
    "                        self._pvx = (1 - a) * self._pvx + \\\n"
    "                            a * (px - self._pv_x) / dtp\n"
    "                        self._pvy = (1 - a) * self._pvy + \\\n"
    "                            a * (py - self._pv_y) / dtp\n"
    "            if _v_inst is None or _v_inst > 2.0 or (now - self._pv_t) > 0.60:\n"
    "                self._pv_t, self._pv_x, self._pv_y = now, px, py\n"
    "            else:\n"
    "                self._pv_t, self._pv_x, self._pv_y = now, px, py\n",

    "            # P73: 窗口最小二乘估人速(见 fit_vel)。跨相机切换会让 px/py\n"
    "            # 瞬间跳一下, 必须整窗作废, 否则拟合出几十 m/s 的假速度;\n"
    "            # 视觉中断超过 0.5s 也要作废, 否则新旧样本跨空档被连成一条线。\n"
    "            if self._pv_cam != vis_cam:\n"
    "                self._pv_cam = vis_cam\n"
    "                self._pv_win = []\n"
    "            if self._pv_win and now - self._pv_win[-1][0] > 0.50:\n"
    "                self._pv_win = []\n"
    "            self._pv_win.append((now, px, py))\n"
    "            while len(self._pv_win) > 1 and \\\n"
    "                    now - self._pv_win[0][0] > 1.20:\n"
    "                self._pv_win.pop(0)\n"
    "            if now - self._pv_win[0][0] > 0.35:\n"
    "                _v = fit_vel(self._pv_win)\n"
    "                if _v is not None and math.hypot(_v[0], _v[1]) < 2.5:\n"
    "                    self._pvx, self._pvy = _v\n"
    "            self._pv_t = now\n",
))

# ---------- E. section 6: 去掉静默否决 ----------
EDITS.append((
    "                _dpath, _ok6, _obj6 = path_free(rx, ry, ryaw)\n"
    "                if _dpath < 0.33 or not _ok6:\n"
    "                    v_t = 0.0\n"
    "                    if isinstance(_obj6, str):\n"
    "                        note = (note + ' 墙前硬停').strip()\n"
    "                    elif _obj6 is not None:\n",

    "                _dpath, _ok6, _obj6, _s6 = path_free(rx, ry, ryaw)\n"
    "                # P73 关键修复: 原来是 `if _dpath < 0.33 or not _ok6`。\n"
    "                # _ok6 是\"沿 1.6m 扫掠\"的布尔量, 只要沿途任一点车心到\n"
    "                # 障碍表面 <= ROBOT_R+0.10 = 0.28m 就为 False; 而当没有\n"
    "                # 真正接触时 _obj6 是 None → 既不写 note 也不给转向, v_t\n"
    "                # 被静默改成 0。实测: 目标 2.44m、正前方 1.48m 无遮挡、\n"
    "                # ff/pid/cap 全部正常, 但 dh 恒为 0.00、指令 v 恒为 +0.00,\n"
    "                # 120s 里 34.8% 的采样被这一条掐成 0 → 人越走越远。\n"
    "                # 0.18m 车体半径 + 0.10m 余量过 0.25m 净空完全够, 不该停;\n"
    "                # 现在只在\"真挡住\"(接触距离 <0.33m 或窄段前沿 <0.30m)时\n"
    "                # 硬停, 其余窄道只限速。\n"
    "                if _dpath < 0.33 or _s6 < 0.30:\n"
    "                    v_t = 0.0\n"
    "                    if isinstance(_obj6, str):\n"
    "                        note = (note + ' 墙前硬停').strip()\n"
    "                    elif _obj6 is None:\n"
    "                        note = (note + ' 窄道硬停 前沿%.2f' % _s6).strip()\n"
    "                    elif _obj6 is not None:\n",
))

# ---------- F. 安全层: 去掉 d_head=0 一刀切 ----------
EDITS.append((
    "        _dpath, _okp, _objp = path_free(self.rx, self.ry, self.ryaw)\n"
    "        if self._avoiding:\n"
    "            d_head = d_f             # 绕障时车头已偏向侧方, 看正前方即可\n"
    "        elif not _okp:\n"
    "            d_head = 0.0             # 走廊太窄(贴角) → 不许再往前挤\n"
    "        else:\n"
    "            d_head = min(d_f, _dpath)\n",

    "        _dpath, _okp, _objp, _stight = path_free(self.rx, self.ry, self.ryaw)\n"
    "        if self._avoiding:\n"
    "            d_head = d_f             # 绕障时车头已偏向侧方, 看正前方即可\n"
    "        else:\n"
    "            # P73: 原 `elif not _okp: d_head = 0.0` 与 section 6 同一个病\n"
    "            # —— 把\"旁边有柱子\"当成\"前方堵死\"。硬闸改用真实接触距离\n"
    "            # _dpath; 只有当窄段前沿已贴到车头前 30cm 内才收紧。\n"
    "            d_head = min(d_f, _dpath)\n"
    "            if _stight < 0.30:\n"
    "                d_head = min(d_head, _stight)\n",
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
sftp.close()

print('已打补丁 %d 处' % len(EDITS))
i, o, e = cli.exec_command(
    'cd %s && python3 -m py_compile %s && echo PY_OK' % (HOME, SRC),
    timeout=60)
print(o.read().decode(), e.read().decode()[:500])
i, o, e = cli.exec_command(
    'grep -c "path_free(" %s; md5sum %s %s' % (SRC, SRC, DEP), timeout=30)
print(o.read().decode())
cli.close()
