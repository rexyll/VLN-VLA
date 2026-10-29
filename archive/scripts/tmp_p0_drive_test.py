# -*- coding: utf-8 -*-
"""底盘物理直测: 绕过 Nav2/follower, 直接给 /robot/cmd_vel 看机器人能不能走。

目的: 区分"Nav2 链路没通"和"SDF 物理坏了(轮轴方向/无 joint)"。
"""
import math
import re

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
                print('[ERR]', err.rstrip()[:300])
        return out

    def pose():
        out = sh('bash -lc "%stimeout 15 ros2 topic echo /odom --once '
                 '--csv 2>/dev/null"' % ENV, quiet=True)
        xs = re.findall(r'-?\d+\.\d+(?:e-?\d+)?', out)
        if len(xs) < 4:
            return None
        # --csv 把 position(x,y,z) 和 orientation(x,y,z,w) 平铺, 顺序不定,
        # 这里改用 /odom 的 position 字段单独取(见 pose_xy)
        return None

    def pose_xy():
        out = sh('bash -lc "%stimeout 15 ros2 topic echo /odom --once '
                 '--field pose.pose.position --csv 2>/dev/null"' % ENV,
                 quiet=True)
        xs = re.findall(r'-?\d+\.\d+(?:e-?\d+)?', out)
        return (float(xs[0]), float(xs[1])) if len(xs) >= 2 else None

    def yaw():
        out = sh('bash -lc "%stimeout 15 ros2 topic echo /odom --once '
                 '--field pose.pose.orientation --csv 2>/dev/null"' % ENV,
                 quiet=True)
        xs = re.findall(r'-?\d+\.\d+(?:e-?\d+)?', out)
        if len(xs) < 4:
            return None
        x, y, z, w = (float(v) for v in xs[:4])
        return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

    import io
    import os
    sftp = cli.open_sftp()
    local = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         '.stage_p0', 'p0_drive_prep.sh')
    with sftp.file('/tmp/p0_drive_prep.sh', 'wb') as f:
        f.write(io.open(local, 'rb').read())
    sftp.chmod('/tmp/p0_drive_prep.sh', 0o755)
    sftp.close()

    print('--- 停掉所有写 /robot/cmd_vel 的节点(cmd_mux 会 20Hz 覆盖我的指令) ---')
    sh('bash /tmp/p0_drive_prep.sh')

    p0 = pose_xy()
    y0 = yaw()
    print('起点 (%+.3f, %+.3f) yaw=%.1f°'
          % (p0[0], p0[1], math.degrees(y0)))

    print('\n--- A) 直行: linear.x=0.30 持续 3s ---')
    sh('bash -lc \'%stimeout 3 ros2 topic pub -r 20 /robot/cmd_vel '
       'geometry_msgs/msg/Twist "{linear: {x: 0.30}}" > /dev/null 2>&1; '
       "sleep 1' " % ENV)
    p1 = pose_xy()
    d1 = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
    print('位移 %.3fm  (理想 ~0.90m, 前进方向投影 %.3fm)'
          % (d1, math.cos(y0) * (p1[0] - p0[0]) + math.sin(y0) * (p1[1] - p0[1])))

    print('\n--- B) 原地转: angular.z=0.80 持续 2s ---')
    sh('bash -lc \'%stimeout 2 ros2 topic pub -r 20 /robot/cmd_vel '
       'geometry_msgs/msg/Twist "{angular: {z: 0.80}}" > /dev/null 2>&1; '
       "sleep 1' " % ENV)
    p2 = pose_xy()
    y2 = yaw()
    dy = math.degrees(math.atan2(math.sin(y2 - y0), math.cos(y2 - y0)))
    print('yaw 变化 %.1f°  (理想 ~91.7°)   平移漂移 %.3fm'
          % (dy, math.hypot(p2[0] - p1[0], p2[1] - p1[1])))

    print('\n--- 结论判据 ---')
    print('直行位移 <0.15m 而原地转 yaw 正常 → 轮轴方向错(SDF 物理问题), '
          '不是 Nav2 链路问题')

    print('\n--- SDF 轮轴检查 ---')
    sh("grep -n -A3 'left_wheel_joint' /home/earl/workspace/VLA/vla_ws/install/"
       'vla_follow/share/vla_follow/worlds/work_area.sdf | grep -nE "axis|xyz" '
       '| head -6')
    sh("awk '/<joint name=\"left_wheel_joint\"/,/</joint>/' /home/earl/workspace/"
       'VLA/vla_ws/install/vla_follow/share/vla_follow/worlds/work_area.sdf '
       "| head -20")
    cli.close()


if __name__ == '__main__':
    main()
