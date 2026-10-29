# -*- coding: utf-8 -*-
"""P0 独立验证: Nav2 -> cmd_mux -> 底盘 这条链是否真的让机器人动。

停掉 follower(避免视觉伺服抢控制权) → 手动把 /nav2_active 置 True
→ 下发 1.2m 的 NavigateToPose 目标 → 比对 /odom 起止点。
"""
import io
import math
import os
import re
import time

import paramiko

HOST, USER, PWD = '192.168.255.128', 'earl', '111111'
HOME = '/home/earl/workspace/VLA'
ENV = ("source /opt/ros/jazzy/setup.bash && "
       "source %s/vla_ws/install/setup.bash && " % HOME)


def main():
    cli = paramiko.SSHClient()
    cli.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    cli.connect(HOST, 22, USER, PWD, timeout=10,
                look_for_keys=False, allow_agent=False)

    def sh(cmd, t=120, quiet=False):
        i, o, e = cli.exec_command(cmd, timeout=t)
        out = o.read().decode('utf-8', 'replace')
        err = e.read().decode('utf-8', 'replace')
        if not quiet:
            if out.strip():
                print(out.rstrip())
            if err.strip():
                print('[ERR]', err.rstrip()[:400])
        return out

    # 把 prep 脚本推上去(独立文件执行, 避免 pkill -f 自匹配)
    sftp = cli.open_sftp()
    local = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         '.stage_p0', 'p0_nav2_prep.sh')
    with sftp.file('/tmp/p0_nav2_prep.sh', 'wb') as f:
        f.write(io.open(local, 'rb').read())
    sftp.chmod('/tmp/p0_nav2_prep.sh', 0o755)
    sftp.close()

    def pose():
        out = sh('bash -lc "%stimeout 15 ros2 topic echo /odom --once '
                 '--field pose.pose.position --csv 2>/dev/null"' % ENV,
                 quiet=True)
        xs = re.findall(r'-?\d+\.\d+(?:e-?\d+)?', out)
        return (float(xs[0]), float(xs[1])) if len(xs) >= 2 else None

    p0 = pose()
    if p0 is None:
        raise SystemExit('取不到 /odom 位姿, 中止')
    print('起点 odom: (%.3f, %.3f)' % p0)

    d = math.hypot(*p0)
    if d < 1.6:
        tx, ty = p0[0], p0[1] - 1.2
    else:
        k = (d - 1.2) / d
        tx, ty = p0[0] * k, p0[1] * k
    dist = math.hypot(tx - p0[0], ty - p0[1])
    print('目标 map:   (%.3f, %.3f)   直线距离 %.2fm' % (tx, ty, dist))

    print('\n--- 1+2) 停 follower, 手动置 /nav2_active=True ---')
    sh('bash /tmp/p0_nav2_prep.sh')
    sh('tail -3 /tmp/nav2act.log')
    sh("grep -E 'CMD_MUX 模式' %s/vla_ws/logs/demo.log | tail -2" % HOME)

    print('\n--- 3) 下发 NavigateToPose ---')
    goal = ('{pose: {header: {frame_id: map}, pose: {position: '
            '{x: %.3f, y: %.3f}, orientation: {w: 1.0}}}}' % (tx, ty))
    sh('bash -lc \'%stimeout 60 ros2 action send_goal /navigate_to_pose '
       'nav2_msgs/action/NavigateToPose "%s" 2>&1 | tail -10\''
       % (ENV, goal), t=120)

    time.sleep(3)
    p1 = pose()
    if p1:
        print('\n终点 odom: (%.3f, %.3f)' % p1)
        print('实际位移 %.3fm (目标 %.2fm), 到目标残差 %.3fm'
              % (math.hypot(p1[0] - p0[0], p1[1] - p0[1]), dist,
                 math.hypot(tx - p1[0], ty - p1[1])))
    print('\n--- demo.log ---')
    sh("grep -E 'CMD_MUX|Begin navigating|Received a goal|Goal (succeeded|"
       "aborted|failed|canceled)' %s/vla_ws/logs/demo.log | tail -14 "
       '| cut -c1-150' % HOME)
    sh("pkill -f 'nav2_act[i]ve' 2>/dev/null; echo CLEANUP_OK")
    cli.close()


if __name__ == '__main__':
    main()
