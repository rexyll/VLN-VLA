"""P75: person_walk 开环录制/回放 (PW_RECORD / PW_REPLAY)
让"人"变成时间的纯函数, 与跟随控制器彻底解耦 —— 这是做 A/B 对照的前提。

P74 已经把时间源切到 /clock, 两轮同种子的轨迹最大偏差从 11.39m 降到 0.573m,
但 14.7s 之后仍缓慢分叉。原因有二:
  (1) 人是闭环的: robot_close(<0.70m) 触发 ESCAPE; 前方 1.5s 预测点离机器人
      <1.3m 就转向。=> 改机器人代码必然改变人的路径, 种子固定也没用。
  (2) gz 按 wall clock 步进(RTF 波动), 定时器抖动让 dt 积分漂移。

PW_REPLAY 直接消灭 (1) 与 (2): 按录制时间轴开环下发 (v, w), 完全不看机器人。
  录制: PW_RECORD=/tmp/walk.jsonl PW_SEED=7 ros2 launch ...
  回放: PW_REPLAY=/tmp/walk.jsonl               ros2 launch ...
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

if 'PW_REPLAY' in s:
    print('SKIP: P75 已存在 (PW_REPLAY found)')
else:
    # ---------------------------------------------------------------- 1) imports
    s = rep1(s,
             "import math\nimport random\nimport threading\nimport time\n",
             "import json\nimport math\nimport os\nimport random\nimport threading\n"
             "import time\n",
             'imports')

    # ------------------------------------------------------- 2) __init__ 模式判定
    s = rep1(s,
             """        self.pub = self.create_publisher(Twist, '/person/cmd_vel', 10)
        self.create_subscription(Odometry, '/person/odom', self._odom_cb, 10)
        self.create_subscription(Odometry, '/robot/odom', self._robot_cb, 10)
""",
             """        self.pub = self.create_publisher(Twist, '/person/cmd_vel', 10)
        self.create_subscription(Odometry, '/person/odom', self._odom_cb, 10)
        self.create_subscription(Odometry, '/robot/odom', self._robot_cb, 10)
        # ---- P75: 开环回放 / 录制 (A/B 对照用) ----
        # 回放模式把"人"变成时间的纯函数: 按录制轴下发 (v,w), 完全不看机器人。
        # 录制模式记录每拍实际下发的 (tau, v, w), tau 以本节点仿真时钟起点为零。
        self._rec_fp = None
        self._rp = None
        self._rp_i = 0
        self._rp_off = 0.0
        self._rp_log = -9.0
        self._rec_path = os.environ.get('PW_RECORD', '').strip()
        self._rp_path = os.environ.get('PW_REPLAY', '').strip()
        if self._rec_path and self._rp_path:
            print('[PERSON] PW_RECORD 与 PW_REPLAY 同时设置, 忽略 PW_RECORD',
                  flush=True)
            self._rec_path = ''
        if self._rec_path:
            self._rec_fp = open(self._rec_path, 'w', encoding='utf-8')
            self._rec_fp.write(json.dumps({
                'hdr': 1, 'seed': os.environ.get('PW_SEED', ''),
                'rate_hz': 20.0}) + '\\n')
            self.get_logger().info('PERSON 录制模式 -> %s' % self._rec_path)
        if self._rp_path:
            self._rp = self._load_replay(self._rp_path)
            try:
                self._rp_off = float(os.environ.get('PW_REPLAY_OFFSET', '0') or 0.0)
            except ValueError:
                self._rp_off = 0.0
            self.get_logger().info(
                'PERSON 回放模式 <- %s (%d 拍, offset=%.3fs)'
                % (self._rp_path, len(self._rp), self._rp_off))
""",
             'init')

    # ------------------------------------- 3) _load_replay / _replay_tick 两个方法
    s = rep1(s,
             "    def _pick_walk(self):\n",
             """    # -------------------------------------------------- P75 录制/回放
    def _load_replay(self, path):
        \"\"\"读回录制文件, 返回按 tau 升序的 [(tau, v, w), ...]。\"\"\"
        seq = []
        try:
            with open(path, 'r', encoding='utf-8') as f:
                for ln in f:
                    ln = ln.strip()
                    if not ln:
                        continue
                    try:
                        o = json.loads(ln)
                    except Exception:
                        continue
                    if 'hdr' in o:
                        continue
                    if 't' in o and 'v' in o and 'w' in o:
                        seq.append((float(o['t']), float(o['v']),
                                    float(o['w'])))
        except IOError as e:
            print('[PERSON] 回放文件读取失败: %s' % e, flush=True)
        seq.sort(key=lambda r: r[0])
        return seq

    def _replay_tick(self):
        \"\"\"开环回放: 只用 tau 查表, 不看机器人位置/障碍, 不跑状态机。\"\"\"
        now = self._now() - self._rp_off
        seq = self._rp
        if not seq:
            self._publish(0.0, 0.0)
            return
        if now < seq[0][0] or now > seq[-1][0] + 2.0:
            # 录制轴之前/之后: 静止, 保证时间轴严格对齐
            self._publish(0.0, 0.0)
            return
        while self._rp_i + 1 < len(seq) and seq[self._rp_i + 1][0] <= now:
            self._rp_i += 1
        i = self._rp_i
        t0, v0, w0 = seq[i]
        v, w = v0, w0
        if i + 1 < len(seq):
            t1, v1, w1 = seq[i + 1]
            if t1 > t0:
                a = (now - t0) / (t1 - t0)
                a = 0.0 if a < 0.0 else (1.0 if a > 1.0 else a)
                v = v0 + (v1 - v0) * a
                w = w0 + (w1 - w0) * a
        self._publish(v, w)
        if now - self._rp_log > 2.0:
            self._rp_log = now
            x, y, yaw = self._pos()
            self.get_logger().info(
                'PERSON(回放) tau=%.2fs v=%.3f w=%+.3f 位置=(%+.2f, %+.2f)'
                % (now, v, w, x, y))

    def _pick_walk(self):
""",
             'methods')

    # ------------------------------------------------- 4) _tick 入口分流
    s = rep1(s,
             "    def _tick(self):\n        now = self._now()\n",
             """    def _tick(self):
        if self._rp is not None:
            self._replay_tick()
            return
        now = self._now()
""",
             'tick')

    # ------------------------------------------------- 5) _publish 内录制
    s = rep1(s,
             """    def _publish(self, v, w):
        msg = Twist()
""",
             """    def _publish(self, v, w):
        # P75: 录制实际下发量 —— 放在 _publish 里, 保证录到的就是执行量
        if self._rec_fp is not None:
            try:
                self._rec_fp.write(json.dumps({
                    't': round(self._now(), 4),
                    'v': round(float(v), 4),
                    'w': round(float(w), 4)}) + '\\n')
                self._rec_fp.flush()
            except Exception:
                pass
        msg = Twist()
""",
             'publish')

    # ------------------------------------------------- 6) main 收尾关文件
    s = rep1(s,
             """    finally:
        node.destroy_node()
        rclpy.try_shutdown()
""",
             """    finally:
        try:
            if getattr(node, '_rec_fp', None):
                node._rec_fp.flush()
                node._rec_fp.close()
                print('[PERSON] 录制文件已关闭', flush=True)
        except Exception:
            pass
        node.destroy_node()
        rclpy.try_shutdown()
""",
             'main')

    wr(P, s)
    print('PATCHED', P)

    # 语法自检
    i, o, e = cli.exec_command(
        'cd %s && python3 -m py_compile person_walk.py && echo SYNTAX_OK' % SRC,
        timeout=60)
    print(o.read().decode(errors='replace').strip(),
          e.read().decode(errors='replace')[:400])

    # 同步 deploy
    i, o, e = cli.exec_command('cp %s %sperson_walk.py && echo SYNCED' % (P, DEP),
                               timeout=30)
    print(o.read().decode(errors='replace').strip())

sftp.close()
cli.close()
print('DONE')
