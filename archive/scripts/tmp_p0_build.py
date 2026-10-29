# -*- coding: utf-8 -*-
"""P0: 打补丁 start_demo.sh(去掉双 gz 实例) → 停旧实例 → colcon build"""
import io
import posixpath

import paramiko

HOST, USER, PWD = '192.168.255.128', 'earl', '111111'
HOME = '/home/earl/workspace/VLA'


def main():
    cli = paramiko.SSHClient()
    cli.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    cli.connect(HOST, 22, USER, PWD, timeout=10,
                look_for_keys=False, allow_agent=False)

    def run(cmd, t=900, echo=True):
        _i, o, e = cli.exec_command(cmd, timeout=t)
        out = o.read().decode('utf-8', 'replace')
        err = e.read().decode('utf-8', 'replace')
        if echo:
            if out.strip():
                print(out.rstrip())
            if err.strip():
                print('[ERR]', err.rstrip()[:3000])
        return out, err

    # --- 1. start_demo.sh: 不再让 launch 起第二个 gz ---
    sftp = cli.open_sftp()
    p = posixpath.join(HOME, 'vla_demo/start_demo.sh')
    with sftp.file(p, 'r') as f:
        txt = f.read().decode('utf-8')
    if 'start_gz:=false' in txt:
        print('start_demo.sh 已含 start_gz:=false, 跳过')
    else:
        old = 'setsid nohup ros2 launch vla_follow demo.launch.py "$@"'
        new = ('setsid nohup ros2 launch vla_follow demo.launch.py '
               'start_gz:=false "$@"')
        if old not in txt:
            raise SystemExit('未找到 launch 行, 无法打补丁')
        txt = txt.replace(old, new)
        with sftp.file(p, 'w') as f:
            f.write(txt.encode('utf-8'))
        print('start_demo.sh 已补丁: start_gz:=false (避免双 Gazebo 实例)')
    sftp.close()
    run('grep -n "start_gz" %s/vla_demo/start_demo.sh' % HOME)

    # --- 2. 停旧实例 ---
    print('\n=== 停旧实例 ===')
    run("pkill -f 'ros2 launch vla_follow'; pkill -f 'vla_follow/lib/vla_follow'; "
        "pkill -f parameter_bridge; pkill -f 'nav2_'; pkill -f 'gz sim'; "
        "pkill -f 'gz-sim'; sleep 2; "
        "pkill -9 -f 'gz sim'; pkill -9 -f 'vla_follow/lib/vla_follow'; "
        "pkill -9 -f 'nav2_'; sleep 1; "
        "echo '残留: '$(pgrep -c -f 'vla_follow|nav2_|gz sim' 2>/dev/null || echo 0)")
    run("rm -f /tmp/vla_start_demo.lock")

    # --- 3. colcon build ---
    print('\n=== colcon build ===')
    run('source /opt/ros/jazzy/setup.bash; cd %s/vla_ws && '
        'colcon build --packages-select vla_follow '
        '--event-handlers console_direct+ 2>&1 | tail -25' % HOME)

    # --- 4. 校验安装产物 ---
    print('\n=== 安装产物校验 ===')
    run('cd %s && for f in gt_tf cmd_mux follower; do '
        'printf "%%-10s src=%%s install=%%s\\n" $f '
        '$(md5sum vla_ws/src/vla_follow/vla_follow/$f.py|cut -c1-8) '
        '$(md5sum vla_ws/install/vla_follow/lib/python3.12/site-packages/'
        'vla_follow/$f.py|cut -c1-8); done; '
        'echo "--- launch ---"; ls -la --time-style=long-iso '
        'vla_ws/install/vla_follow/share/vla_follow/launch/' % HOME)
    cli.close()


if __name__ == '__main__':
    main()
