# -*- coding: utf-8 -*-
"""P72 底盘标定 v2

- 用 /world/follow_world/pose/info 的**四元数**直接算 robot 的 roll/pitch/yaw
  (不用 gz model -p 的 RPY 打印, 免掉欧拉角分解歧义)
- 用真值位姿的差分找"速度平台段", 算实际线速度/角速度对指令的增益
"""
import io
import math
import os
import posixpath
import time

import paramiko

HOST, USER, PWD = '192.168.255.128', 'earl', '111111'
HOME = '/home/earl/workspace/VLA'
ENV = ("source /opt/ros/jazzy/setup.bash && "
       "source %s/vla_ws/install/setup.bash && " % HOME)
LOCAL = os.path.dirname(os.path.abspath(__file__))

PREP = r'''#!/bin/bash
pkill -f 'vla_follow/lib/vla_follow/follow[e]r'
pkill -f 'vla_follow/lib/vla_follow/cmd_mu[x]'
pkill -f 'vla_follow/lib/vla_follow/person_wal[k]'
pkill -f 'vla_follow/lib/vla_follow/voice_[nt]'
pkill -f 'nav2_act[i]ve'
sleep 1
export DISPLAY=:0
gz service -s /world/follow_world/set_pose --reqtype gz.msgs.Pose \
  --reptype gz.msgs.Boolean --timeout 2000 \
  --req 'name: "person" position: {x: 0 y: 5.0 z: 0}' >/dev/null 2>&1
gz service -s /world/follow_world/set_pose --reqtype gz.msgs.Pose \
  --reptype gz.msgs.Boolean --timeout 2000 \
  --req 'name: "robot" position: {x: 0 y: 0 z: 0} orientation: {w: 1}' >/dev/null 2>&1
sleep 2
echo PREP_DONE
'''

POSE_PY = r'''
import sys
txt = open('/tmp/p72_pose.txt').read()
# 按 "pose {" 切块, 每块只解析自己的 name/position/orientation
blocks = []
cur = None
for line in txt.split('\n'):
    s = line.strip()
    if s == 'pose {':
        if cur is not None:
            blocks.append(cur)
        cur = []
        continue
    if cur is not None:
        cur.append(s)
if cur is not None:
    blocks.append(cur)

out = {}
for b in blocks:
    name = None
    px = py = pz = 0.0
    qx = qy = qz = 0.0
    qw = 1.0
    mode = None
    for t in b:
        if t.startswith('name: "'):
            name = t.split('"')[1]
        elif t.startswith('position {'):
            mode = 'p'
        elif t.startswith('orientation {'):
            mode = 'o'
        elif t == '}':
            mode = None
        elif ':' in t and name in ('robot', 'person'):
            k, v = t.split(':', 1)
            k = k.strip()
            try:
                fv = float(v)
            except ValueError:
                continue
            if mode == 'p':
                if k == 'x': px = fv
                elif k == 'y': py = fv
                elif k == 'z': pz = fv
            elif mode == 'o':
                if k == 'x': qx = fv
                elif k == 'y': qy = fv
                elif k == 'z': qz = fv
                elif k == 'w': qw = fv
    if name in ('robot', 'person'):
        out[name] = (px, py, pz, qx, qy, qz, qw)
for n, v in out.items():
    print('%-7s pos %+.3f %+.3f %+.4f  quat %.5f %.5f %.5f %.5f'
          % (n, v[0], v[1], v[2], v[3], v[4], v[5], v[6]))
'''


def rpy(q):
    x, y, z, w = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    sp = max(-1.0, min(1.0, 2 * (w * y - z * x)))
    pitch = math.asin(sp)
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def main():
    cli = paramiko.SSHClient()
    cli.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    cli.connect(HOST, 22, USER, PWD, timeout=10,
                look_for_keys=False, allow_agent=False)
    sftp = cli.open_sftp()

    def sh(cmd, t=180, quiet=True):
        i, o, e = cli.exec_command(cmd, timeout=t)
        out = o.read().decode('utf-8', 'replace')
        err = e.read().decode('utf-8', 'replace')
        if not quiet:
            if out.strip():
                print(out.rstrip())
            if err.strip():
                print('[ERR]', err.rstrip()[:400])
        return out

    def put(local_name, remote):
        with sftp.file(remote, 'wb') as f:
            f.write(io.open(posixpath.join(LOCAL, local_name), 'rb').read())
        sftp.chmod(remote, 0o755)

    def wpose():
        sh('export DISPLAY=:0; timeout 12 gz topic -e -t '
           '/world/follow_world/pose/info -n 1 > /tmp/p72_pose.txt 2>/dev/null')
        with sftp.file('/tmp/p72_pose.py', 'w') as f:
            f.write(POSE_PY.encode('utf-8'))
        out = sh('python3 /tmp/p72_pose.py', t=30)
        res = {}
        for line in out.splitlines():
            p = line.replace('pos', '').replace('quat', '').split()
            if len(p) >= 8:
                res[p[0]] = tuple(float(v) for v in p[1:8])
        return res

    def series(remote):
        try:
            with sftp.file(remote, 'r') as f:
                txt = f.read().decode('utf-8')
        except IOError:
            return []
        rows = []
        for line in txt.splitlines():
            p = line.split()
            if len(p) == 4:
                try:
                    rows.append(tuple(float(v) for v in p))
                except ValueError:
                    pass
        return rows

    put('p72_sample.py', '/tmp/p72_sample.py')
    with sftp.file('/tmp/p72_prep.sh', 'wb') as f:
        f.write(PREP.encode('utf-8'))
    sftp.chmod('/tmp/p72_prep.sh', 0o755)

    print('=== 1. 准备 ===')
    sh('bash /tmp/p72_prep.sh; echo done', quiet=False)
    w = wpose()
    for n in ('robot', 'person'):
        if n in w:
            r_, p_, y_ = rpy(w[n][3:])
            print('%-7s pos %+.3f %+.3f %+.4f   roll %+.4f  pitch %+.4f (%.1f°)  '
                  'yaw %+.4f' % (n, w[n][0], w[n][1], w[n][2], r_, p_,
                                 math.degrees(p_), y_))

    print('\n=== 2. 直行 linear.x=+0.20 (60 帧@20Hz) ===')
    sh('rm -f /tmp/p72_a.txt; setsid nohup bash -c "%spython3 /tmp/p72_sample.py '
       '/tmp/p72_a.txt" >/dev/null 2>&1 < /dev/null & sleep 3' % ENV, t=30)
    sh('bash -lc \'%stimeout 15 ros2 topic pub -r 20 --times 60 /robot/cmd_vel '
       'geometry_msgs/msg/Twist "{linear: {x: 0.20}}" >/dev/null 2>&1; '
       'timeout 5 ros2 topic pub --once /robot/cmd_vel geometry_msgs/msg/Twist '
       '"{}" >/dev/null 2>&1; sleep 1.5\'' % ENV, t=45)
    sh("pkill -f 'p72_sampl[e]r'; sleep 1")
    wa = wpose()
    if 'robot' in wa:
        r_, p_, y_ = rpy(wa['robot'][3:])
        print('  行驶后 pitch %+.4f (%.1f°)  roll %+.4f' % (p_, math.degrees(p_), r_))

    print('\n=== 3. 原地转 angular.z=+0.50 (40 帧@20Hz) ===')
    sh('rm -f /tmp/p72_b.txt; setsid nohup bash -c "%spython3 /tmp/p72_sample.py '
       '/tmp/p72_b.txt" >/dev/null 2>&1 < /dev/null & sleep 3' % ENV, t=30)
    sh('bash -lc \'%stimeout 15 ros2 topic pub -r 20 --times 40 /robot/cmd_vel '
       'geometry_msgs/msg/Twist "{angular: {z: 0.50}}" >/dev/null 2>&1; '
       'timeout 5 ros2 topic pub --once /robot/cmd_vel geometry_msgs/msg/Twist '
       '"{}" >/dev/null 2>&1; sleep 1.5\'' % ENV, t=45)
    sh("pkill -f 'p72_sampl[e]r'; sleep 1")
    wb = wpose()
    if 'robot' in wb:
        r_, p_, y_ = rpy(wb['robot'][3:])
        print('  转向后 pitch %+.4f (%.1f°)  roll %+.4f  yaw %+.4f'
              % (p_, math.degrees(p_), r_, y_))

    # ---------- 分析 ----------
    def plateau(rows, kind):
        """返回 (中位速度, 平台时长, 段内总位移/转角, 横向漂移)"""
        if len(rows) < 8:
            return None
        vs = []
        for a, b in zip(rows, rows[1:]):
            dt = b[0] - a[0]
            if dt <= 1e-4:
                continue
            if kind == 'lin':
                dyaw = math.atan2(math.sin(b[3] - a[3]), math.cos(b[3] - a[3]))
                # 只用位移长度, 方向由 yaw 决定
                dx, dy = b[1] - a[1], b[2] - a[2]
                v = math.hypot(dx, dy) / dt
                vs.append((v, dt, dyaw, dx, dy))
            else:
                dyaw = math.atan2(math.sin(b[3] - a[3]), math.cos(b[3] - a[3]))
                vs.append((abs(dyaw) / dt, dt, dyaw, b[1] - a[1], b[2] - a[2]))
        if not vs:
            return None
        thr = 0.05 if kind == 'lin' else 0.10
        peak = max(v[0] for v in vs)
        sel = [v for v in vs if v[0] > max(thr, 0.6 * peak)]
        vs_sorted = sorted(v[0] for v in sel)
        med = vs_sorted[len(vs_sorted) // 2]
        dur = sum(v[1] for v in sel)
        return med, dur, peak, len(sel)

    print('\n=== 4. 结果 ===')
    a = series('/tmp/p72_a.txt')
    print('直行采样 %d 点' % len(a))
    if a:
        for row in a[:2] + a[-2:]:
            print('   t=%.2f  x=%+.3f y=%+.3f yaw=%+.3f' % row)
    r = plateau(a, 'lin')
    if r:
        med, dur, peak, n = r
        print('直行: %d 点, 平台中位速度 %.3f m/s (指令 0.20) → 增益 %.2f×,'
              ' 峰值 %.3f' % (n, med, med / 0.20, peak))
        mx = max(abs(row[1]) for row in a)
        my = max(abs(row[2]) for row in a)
        yaws = [row[3] for row in a]
        dy = math.atan2(math.sin(yaws[-1] - yaws[0]), math.cos(yaws[-1] - yaws[0]))
        print('      全程 Δ=(%+.3f, %+.3f), 轨迹包围盒 %.2f×%.2f m, yaw 变化 %+.1f°'
              % (a[-1][1] - a[0][1], a[-1][2] - a[0][2], 2 * mx, 2 * my,
                 math.degrees(dy)))
    b = series('/tmp/p72_b.txt')
    r = plateau(b, 'ang')
    if r:
        med, dur, peak, n = r
        print('原地转: %d 点, 平台中位角速度 %.3f rad/s (指令 0.50) → 增益 %.2f×'
              % (n, med, med / 0.50))

    sftp.close()
    cli.close()


if __name__ == '__main__':
    main()
