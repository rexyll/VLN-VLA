# -*- coding: utf-8 -*-
"""P72 闭环验收: Nav2 NavigateToPose → /cmd_vel_smoothed → cmd_mux → 底盘 → 真值位姿

不经过 follower, 直接给 /nav2_active 置真让 cmd_mux 走 Nav2 通道,
然后下发目标点, 看机器人是否真的走到(用 /odom 真值算终点误差)。
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

GOAL_X, GOAL_Y = 2.50, 0.60


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
                print('[ERR]', err.rstrip()[:500])
        return out

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

    # 采样脚本上传
    with sftp.file('/tmp/p72_sample.py', 'wb') as f:
        f.write(io.open(posixpath.join(LOCAL, 'p72_sample.py'), 'rb').read())
    sftp.chmod('/tmp/p72_sample.py', 0o755)

    print('=== 1. 清掉干扰进程 + 归位 ===')
    sh("pkill -f 'p72_sample'; "
       "pkill -f 'vla_follow/lib/vla_follow/follow[e]r'; "
       "pkill -f 'vla_follow/lib/vla_follow/person_wal[k]'; "
       "pkill -f 'nav2_act[i]ve'; sleep 1; echo ok", quiet=False)
    sh("export DISPLAY=:0; "
       "gz service -s /world/follow_world/set_pose --reqtype gz.msgs.Pose "
       "--reptype gz.msgs.Boolean --timeout 2000 "
       "--req 'name: \"person\" position: {x: 0 y: 5.0 z: 0}' >/dev/null 2>&1; "
       "gz service -s /world/follow_world/set_pose --reqtype gz.msgs.Pose "
       "--reptype gz.msgs.Boolean --timeout 2000 "
       "--req 'name: \"robot\" position: {x: 0 y: 0 z: 0} "
       "orientation: {w: 1}' >/dev/null 2>&1; sleep 2; echo posed", quiet=False)
    print(sh('pgrep -fc "cmd_mu[x]" || echo "CMD_MUX_MISSING"'))

    print('\n=== 2. 打开 Nav2 通道(/nav2_active=true) ===')
    sh('setsid nohup bash -c "%stimeout 75 ros2 topic pub -r 5 /nav2_active '
       'std_msgs/msg/Bool \\"{data: true}\\"" >/dev/null 2>&1 </dev/null & '
       'sleep 2' % ENV, t=30)
    print(sh('grep -c "CMD_MUX 模式" %s/vla_ws/logs/demo.log' % HOME))

    print('\n=== 3. 采样 + 下发目标 (%.2f, %.2f) ===' % (GOAL_X, GOAL_Y))
    sh('rm -f /tmp/p72_goto.txt; setsid nohup bash -c "%spython3 '
       '/tmp/p72_sample.py /tmp/p72_goto.txt" >/dev/null 2>&1 </dev/null & '
       'sleep 2' % ENV, t=30)
    out = sh('bash -lc \'%stimeout 70 ros2 action send_goal /navigate_to_pose '
             'nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, '
             'pose: {position: {x: %.2f, y: %.2f, z: 0.0}, '
             'orientation: {w: 1.0}}}}" 2>&1 | tail -6\'' % (ENV, GOAL_X, GOAL_Y),
             t=90, quiet=False)
    sh("pkill -f 'p72_sample'; sleep 1")

    rows = series('/tmp/p72_goto.txt')
    print('\n=== 4. 结果 (%d 点) ===' % len(rows))
    if rows:
        x0, y0 = rows[0][1], rows[0][2]
        x1, y1 = rows[-1][1], rows[-1][2]
        err = math.hypot(x1 - GOAL_X, y1 - GOAL_Y)
        print('起点 (%+.3f, %+.3f) → 终点 (%+.3f, %+.3f)' % (x0, y0, x1, y1))
        print('目标 (%.2f, %.2f): 终点误差 %.3f m' % (GOAL_X, GOAL_Y, err))
        path = sum(math.hypot(b[1] - a[1], b[2] - a[2])
                   for a, b in zip(rows, rows[1:]))
        print('路径长度 %.2f m (直线 %.2f m), 用时 %.1fs(sim)'
              % (path, math.hypot(GOAL_X - x0, GOAL_Y - y0),
                 rows[-1][0] - rows[0][0]))
        # 轨迹抽样
        step = max(1, len(rows) // 14)
        for r in rows[::step]:
            print('   t=%5.1f  (%+.2f, %+.2f)  yaw=%+.2f'
                  % (r[0] - rows[0][0], r[1], r[2], r[3]))

    print('\n=== 5. cmd_mux / Nav2 日志 ===')
    sh('grep -E "CMD_MUX 模式|硬闸" %s/vla_ws/logs/demo.log | tail -8 | cut -c1-120'
       % HOME, quiet=False)
    sftp.close()
    cli.close()


if __name__ == '__main__':
    main()
