"""P75b: 回放 tau 轴锚定绝对仿真时间(去掉节点启动抖动)

P75 首版把 tau 锚在 person_walk 节点自己的启动时刻(_now() 以节点起点为零)。
但节点启动的仿真时刻本身有 2.6s 抖动(实测 r2 人的起步绝对时刻 12.7s,
r3 是 15.3s) —— 于是两次回放虽然下发同一串 (v,w), 却发生在世界时间轴的不同
位置: 机器人多跑了 2.6s 才等到人动, 状态不同 => 回放仍不可比。

改法: 录制时写绝对仿真时间(gz world 从 0 起, 每次启动都是 0), 回放用
tau = now_abs - hdr.t0_abs。这样人的起步绝对时刻在所有轮次完全相同,
与节点何时启动无关。
"""
import sys

import paramiko

HOME = '/home/earl/workspace/VLA'
SRC = HOME + '/vla_ws/src/vla_follow/vla_follow/'
DEP = HOME + '/deploy/'

cli = paramiko.SSHClient()
cli.set_missing_host_key_policy(paramiko.AutoAddPolicy())
cli.connect('192.168.255.128', 22, 'earl', '111111',
            timeout=10, look_for_keys=False, allow_agent=False)
sftp = cli.open_sftp()


def rd(p):
    with sftp.file(p, 'r') as f:
        return f.read().decode('utf-8')


def wr(p, s):
    with sftp.file(p, 'w') as f:
        f.write(s.encode('utf-8'))


def rep1(s, old, new, tag):
    n = s.count(old)
    if n != 1:
        print('FAIL[%s] count=%d' % (tag, n))
        sys.exit(1)
    return s.replace(old, new, 1)


P = SRC + 'person_walk.py'
s = rd(P)

if '_now_abs' in s:
    print('SKIP: P75b 已存在')
else:
    # ------------------------------------------- 1) 新增 _now_abs + 录制头字段
    s = rep1(s,
             """    def _odom_cb(self, msg):
""",
             """    def _now_abs(self):
        \"\"\"P75b: 绝对仿真时间(s)。无仿真时钟时返回 None(该拍不录/不动)。\"\"\"
        try:
            ns = int(self.get_clock().now().nanoseconds)
        except Exception:
            ns = 0
        return ns * 1e-9 if ns > 0 else None

    def _odom_cb(self, msg):
""",
             'now_abs')

    # ------------------------------------------- 2) 录制头改为惰性写 + 带 t0_abs
    s = rep1(s,
             """        if self._rec_path:
            self._rec_fp = open(self._rec_path, 'w', encoding='utf-8')
            self._rec_fp.write(json.dumps({
                'hdr': 1, 'seed': os.environ.get('PW_SEED', ''),
                'rate_hz': 20.0}) + '\\n')
            self.get_logger().info('PERSON 录制模式 -> %s' % self._rec_path)
""",
             """        self._rec_hdr_done = False
        self._rec_t0 = None
        if self._rec_path:
            self._rec_fp = open(self._rec_path, 'w', encoding='utf-8')
            self.get_logger().info('PERSON 录制模式 -> %s' % self._rec_path)
""",
             'rec_hdr')

    # ------------------------------------------- 3) _publish 录制用绝对时间
    s = rep1(s,
             """        if self._rec_fp is not None:
            try:
                self._rec_fp.write(json.dumps({
                    't': round(self._now(), 4),
                    'v': round(float(v), 4),
                    'w': round(float(w), 4)}) + '\\n')
                self._rec_fp.flush()
            except Exception:
                pass
""",
             """        if self._rec_fp is not None:
            try:
                ta = self._now_abs()
                if ta is not None:
                    if not self._rec_hdr_done:
                        self._rec_hdr_done = True
                        self._rec_t0 = ta
                        self._rec_fp.write(json.dumps({
                            'hdr': 1,
                            'seed': os.environ.get('PW_SEED', ''),
                            't0_abs': round(ta, 4),
                            'rate_hz': 20.0}) + '\\n')
                    self._rec_fp.write(json.dumps({
                        't': round(ta, 4),
                        'v': round(float(v), 4),
                        'w': round(float(w), 4)}) + '\\n')
                    self._rec_fp.flush()
            except Exception:
                pass
""",
             'rec_write')

    # ------------------------------------------- 4) _load_replay 返回头信息
    s = rep1(s,
             """        seq.sort(key=lambda r: r[0])
        return seq
""",
             """        seq.sort(key=lambda r: r[0])
        return seq, hdr
""",
             'load_ret')

    s = rep1(s,
             """        seq = []
        try:
            with open(path, 'r', encoding='utf-8') as f:
""",
             """        seq = []
        hdr = None
        try:
            with open(path, 'r', encoding='utf-8') as f:
""",
             'load_hdr')

    s = rep1(s,
             """                    if 'hdr' in o:
                        continue
""",
             """                    if 'hdr' in o:
                        hdr = o
                        continue
""",
             'load_hdr2')

    # ------------------------------------------- 5) 回放: tau 锚绝对时间
    s = rep1(s,
             """            self._rp = self._load_replay(self._rp_path)
""",
             """            self._rp, self._rp_hdr = self._load_replay(self._rp_path)
            self._rp_t0 = None
            self._rp_t0_logged = False
            if self._rp_hdr and 't0_abs' in self._rp_hdr:
                self.get_logger().info(
                    'PERSON 回放头: t0_abs=%.3fs seed=%s'
                    % (float(self._rp_hdr['t0_abs']),
                       self._rp_hdr.get('seed', '')))
            else:
                self.get_logger().warn(
                    'PERSON 回放文件无 t0_abs(旧格式), 回退为节点相对时间轴')
""",
             'rp_load')

    s = rep1(s,
             """        now = self._now() - self._rp_off
        seq = self._rp
""",
             """        ta = self._now_abs()
        if ta is None:
            self._publish(0.0, 0.0)
            return
        if self._rp_t0 is None:
            if self._rp_hdr and 't0_abs' in self._rp_hdr:
                self._rp_t0 = float(self._rp_hdr['t0_abs'])
            else:
                self._rp_t0 = ta
            if not self._rp_t0_logged:
                self._rp_t0_logged = True
                _dl = ta - self._rp_t0
                self.get_logger().info(
                    'PERSON 回放锚点 t0_abs=%.3f 本节点首拍=%.3f (差 %+.2fs)'
                    % (self._rp_t0, ta, _dl))
                if _dl > 0.5:
                    self.get_logger().warn(
                        'PERSON 回放起点晚了 %.2fs, 前段轨迹被跳过' % _dl)
                elif _dl < -0.5:
                    self.get_logger().info(
                        'PERSON 回放提前 %.2fs, 静止等待对齐' % (-_dl))
        now = ta - self._rp_t0 + self._rp_off
        seq = self._rp
""",
             'rp_tau')

    wr(P, s)
    print('PATCHED', P)
    i, o, e = cli.exec_command(
        'cd %s && python3 -m py_compile person_walk.py && echo SYNTAX_OK' % SRC,
        timeout=60)
    print(o.read().decode(errors='replace').strip(),
          e.read().decode(errors='replace')[:400])
    i, o, e = cli.exec_command('cp %s %sperson_walk.py && echo SYNCED' % (P, DEP),
                               timeout=30)
    print(o.read().decode(errors='replace').strip())

sftp.close()
cli.close()
print('DONE')
