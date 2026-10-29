"""P74a: 时间源统一切到 /clock(仿真时间) + dt 实测 —— 修评测不可重复

根因: person_walk 与 follower 全程 time.monotonic(), 而 gz 物理按仿真时间步进、
RTF 有波动。同一 PW_SEED 两次跑, 人的轨迹差 11m, 任何调参都无法 A/B。
/clock 早已桥接(bridge.yml GZ_TO_ROS), gt_tf/cmd_mux 也已在用 use_sim_time。
"""
import sys
import paramiko

HOME = '/home/earl/workspace/VLA'
SRC = HOME + '/vla_ws/src/vla_follow/vla_follow/'
LAUNCH = HOME + '/vla_ws/src/vla_follow/launch/demo.launch.py'
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


# ---------------------------------------------------------------- follower
f = SRC + 'follower.py'
s = rd(f)
n0 = s.count('time.monotonic()')
if 'self._now()' in s:
    print('follower: already patched, skip')
else:
    if n0 != 10:
        print('follower: unexpected monotonic count %d' % n0)
        sys.exit(1)
    s = s.replace('time.monotonic()', 'self._now()')
    # __init__ 增加时间基准
    s = rep1(s,
             "        self.cfg = load_track_cfg()\n",
             "        self.cfg = load_track_cfg()\n"
             "        # P74: 统一时间源 —— 优先 /clock(仿真时间), 回退 wall clock。\n"
             "        # 原来全程 wall clock, 与 gz 的仿真步进(RTF 波动)解耦,\n"
             "        # 同一 PW_SEED 两次跑人的轨迹能差 11m, 评测无法 A/B。\n"
             "        self._t0_wall = time.monotonic()\n"
             "        self._t0_sim = None\n",
             'follower-init')
    # _now 方法插在 _pose_cb 之前
    s = rep1(s,
             "    def _pose_cb(self, msg):\n",
             "    def _now(self):\n"
             "        \"\"\"P74: 当前时间(s)。有 /clock 就用仿真时间(归零到节点启动),\n"
             "        否则回退 wall clock —— 两条分支都从 0 起, 切换时只跳几百 ms。\"\"\"\n"
             "        try:\n"
             "            ns = int(self.get_clock().now().nanoseconds)\n"
             "        except Exception:\n"
             "            ns = 0\n"
             "        if ns > 0:\n"
             "            if self._t0_sim is None:\n"
             "                self._t0_sim = ns * 1e-9\n"
             "            return ns * 1e-9 - self._t0_sim\n"
             "        return time.monotonic() - self._t0_wall\n"
             "\n"
             "    def _pose_cb(self, msg):\n",
             'follower-now')
    wr(f, s)
    print('follower: patched, monotonic %d -> %d (new: _now + _t0_wall)'
          % (n0, s.count('time.monotonic()')))

# ------------------------------------------------------------- person_walk
p = SRC + 'person_walk.py'
s = rd(p)
if 'self._now()' in s:
    print('person_walk: already patched, skip')
else:
    n0 = s.count('time.monotonic()')
    if n0 != 8:
        print('person_walk: unexpected monotonic count %d' % n0)
        sys.exit(1)
    s = s.replace('time.monotonic()', 'self._now()')
    s = rep1(s,
             "        super().__init__('person_walk')\n",
             "        super().__init__('person_walk')\n"
             "        # P74: 时间基准 —— 优先 /clock, 回退 wall clock\n"
             "        self._t0_wall = time.monotonic()\n"
             "        self._t0_sim = None\n"
             "        self._dt_prev = None\n",
             'pw-init')
    s = rep1(s,
             "    def _odom_cb(self, msg):\n",
             "    def _now(self):\n"
             "        \"\"\"P74: 当前时间(s), 仿真时钟优先。\"\"\"\n"
             "        try:\n"
             "            ns = int(self.get_clock().now().nanoseconds)\n"
             "        except Exception:\n"
             "            ns = 0\n"
             "        if ns > 0:\n"
             "            if self._t0_sim is None:\n"
             "                self._t0_sim = ns * 1e-9\n"
             "            return ns * 1e-9 - self._t0_sim\n"
             "        return time.monotonic() - self._t0_wall\n"
             "\n"
             "    def _odom_cb(self, msg):\n",
             'pw-now')
    # dt 实测: 硬编码 0.05 在定时器抖动下会让速度积分漂移
    s = rep1(s,
             "    def _tick(self):\n"
             "        dt = 0.05\n"
             "        now = self._now()\n",
             "    def _tick(self):\n"
             "        now = self._now()\n"
             "        # P74: dt 由时钟实测, 不再硬编码 —— 定时器抖动会让速度\n"
             "        # 积分漂移, 同种子两次跑的轨迹因此发散\n"
             "        dt = 0.05 if self._dt_prev is None else \\\n"
             "            max(0.005, min(0.20, now - self._dt_prev))\n"
             "        self._dt_prev = now\n",
             'pw-tick')
    wr(p, s)
    print('person_walk: patched, monotonic %d -> %d'
          % (n0, s.count('time.monotonic()')))

# ---------------------------------------------------------------- launch
s = rd(LAUNCH)
if s.count("executable='person_walk',\n             output='both'),") == 1:
    s = s.replace(
        "executable='person_walk',\n             output='both'),",
        "executable='person_walk',\n"
        "             output='both', parameters=[{'use_sim_time': True}]),", 1)
    s = s.replace(
        "Node(package='vla_follow', executable='follower', output='both'),",
        "Node(package='vla_follow', executable='follower', output='both',\n"
        "             parameters=[{'use_sim_time': True}]),", 1)
    wr(LAUNCH, s)
    print('launch: use_sim_time added to person_walk + follower')
else:
    print('launch: already patched or anchor mismatch, skip')

# ----------------------------------------------------------------- deploy
for name in ('follower.py', 'person_walk.py'):
    wr(DEP + name, rd(SRC + name))
print('deploy synced')

# ---------------------------------------------------------------- 语法检查
for path in (f, p, LAUNCH):
    _in, out, _e = cli.exec_command('python3 -m py_compile "%s" && echo OK' % path)
    r = out.read().decode().strip()
    print('compile %-22s %s' % (path.split('/')[-1], r or 'FAILED'))

# ---------------------------------------------------------------- md5 校验
_in, out, _e = cli.exec_command(
    'md5sum %sfollower.py %sfollower.py %sperson_walk.py %sperson_walk.py'
    % (SRC, DEP, SRC, DEP))
print(out.read().decode())

sftp.close()
cli.close()
