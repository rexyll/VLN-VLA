"""tests/test_stick.py —— "贴身死锁"确定性复现

摆位: 机器人 (0,0) 朝 +x; 人被固定在机器人**正后方** (-0.42, 0)。
    这正是 p81b_1 退化轮的几何: 人在 0.42m 处、方位 ±112~180°
    (近距离方位估计会跳), 于是
        · 距离环 back_hold/close_back 都要"后方净空", 而后方 0.30m
          那个障碍**就是人自己** → 判据永不成立 → 永不后退;
        · 人同时处在近距盲区(det 时有时无), 扫描只会原地转;
    两者叠加 = 卡在 0.41m 不动(实测 65s)。

判据: 看 ESCP 能否把距离重新拉开。

用法: python3 tmp_stick_test.py <tag> [dur=25]
"""
import json
import math
import os

# 项目根: 产物统一输出到 tests/out/, 不再散落在项目根
VLA_HOME = os.environ.get('VLA_HOME', '/home/earl/workspace/VLA')
OUTDIR = os.path.join(VLA_HOME, 'tests', 'out')
os.makedirs(OUTDIR, exist_ok=True)
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import gz.transport13 as gz_transport
from gz.msgs10.pose_v_pb2 import Pose_V as GzPose_V

WORLD = 'follow_world'
RX, RY = 2.35, 2.00          # 机器人: 立柱 ob1(3.5,2.0, 半宽0.40) 西侧, 朝 -x
RYAW = math.pi               # 朝 -x → 正后方就是 +x
DSTICK = 0.49                # 贴身距离
# 人楔在立柱西侧面(x=3.10)前: 中心 3.10-0.26(裙摆半径)=2.84, 推不动。
# 上一版把人放在空地上, 机器人直接把人顶走, "距离恢复"其实是瞬移复位,
# 测不出任何东西。
PX, PY = 2.84, RY


def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


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


class Stick(Node):
    def __init__(self, tag, dur):
        super().__init__('stick_test')
        self.dur = dur
        self.f = open(os.path.join(OUTDIR, 'stick_%s.jsonl' % tag),
                      'w', encoding='utf-8')
        self.rp = self.pp = None
        self.st = {}
        self.t0 = time.monotonic()
        self.pub_cmd = self.create_publisher(String, '/robot/command', 10)
        self._gz = gz_transport.Node()
        self._gz.subscribe(GzPose_V, '/world/%s/dynamic_pose/info' % WORLD,
                           self._gt)
        self.create_subscription(String, '/follower/status', self._s, 10)
        self.create_timer(0.1, self._tick)
        print('STICK 就绪 tag=%s 人固定在 (%.2f,%.2f)'
              % (tag, PX, PY), flush=True)

    def _gt(self, msg):
        for p in msg.pose:
            if p.name == 'robot':
                self.rp = (p.position.x, p.position.y, yaw_of(p.orientation))
            elif p.name == 'person':
                self.pp = (p.position.x, p.position.y)

    def _s(self, m):
        try:
            self.st = json.loads(m.data)
        except Exception:
            pass

    def _tick(self):
        if self.pp is None or self.rp is None or not self.st:
            return
        px, py = self.pp
        rx, ry, ryaw = self.rp
        d = math.hypot(px - rx, py - ry)
        brg = wrap(math.atan2(py - ry, px - rx) - ryaw)
        self.f.write(json.dumps({
            't': round(time.monotonic() - self.t0, 2),
            'd': round(d, 3), 'brg': round(math.degrees(brg), 1),
            'det': bool(self.st.get('detected')),
            'cam': self.st.get('cam', ''),
            'vd': self.st.get('vis_dist', -1.0),
            'trk': self.st.get('track', ''),
            'mode': self.st.get('mode', ''),
            'rx': round(rx, 2), 'ry': round(ry, 2)}, ensure_ascii=False)
            + '\n')
        self.f.flush()


def main():
    tag = sys.argv[1] if len(sys.argv) > 1 else 's'
    dur = float(sys.argv[2]) if len(sys.argv) > 2 else 25.0
    rclpy.init()
    n = Stick(tag, dur)
    set_pose('robot', RX, RY, 0.15, RYAW)
    set_pose('person', PX, PY, 0.02, 0.0)
    time.sleep(1.5)
    for _ in range(3):
        m = String()
        m.data = json.dumps({'action': 'FOLLOW', 'distance': 1.0})
        n.pub_cmd.publish(m)
        time.sleep(0.4)
    t0 = time.monotonic()
    _fix_t = 0.0
    while time.monotonic() - t0 < dur:
        rclpy.spin_once(n, timeout_sec=0.05)
        # 差速插件的残余速度会把人推走 —— 每 0.8s 复位一次
        if time.monotonic() - _fix_t > 0.8:
            _fix_t = time.monotonic()
            set_pose('person', PX, PY, 0.02, 0.0)
    n.f.close()
    n.destroy_node()
    rclpy.try_shutdown()

    # ---- 汇总 ----
    rows = [json.loads(l) for l in open(os.path.join(OUTDIR,
                                          'stick_%s.jsonl' % tag),
                                        encoding='utf-8')]
    if not rows:
        print('STICK 完成(无数据)', flush=True)
        return
    ds = [r['d'] for r in rows]
    dt = rows[-1]['t'] / max(1, len(rows) - 1)
    stick = sum(1 for x in ds if x < 0.50) * dt
    cont = sum(1 for x in ds if x < 0.44) * dt
    disp = math.hypot(rows[-1]['rx'] - rows[0]['rx'],
                      rows[-1]['ry'] - rows[0]['ry'])
    trks = {}
    for r in rows:
        k = r.get('mode') or r.get('trk') or '?'
        trks[k] = trks.get(k, 0) + 1
    print('STICK 完成 tag=%s  n=%d' % (tag, len(rows)), flush=True)
    print('  距离  起%.2f 最小%.2f 末%.2f 最大%.2f'
          % (ds[0], min(ds), ds[-1], max(ds)), flush=True)
    print('  贴身<0.50m %.1fs   接触<0.44m %.1fs   机器人位移 %.2fm'
          % (stick, cont, disp), flush=True)
    print('  状态占比 %s' % {k: round(100.0 * v / len(rows), 1)
                             for k, v in sorted(trks.items(),
                                                key=lambda x: -x[1])},
          flush=True)


if __name__ == '__main__':
    main()
