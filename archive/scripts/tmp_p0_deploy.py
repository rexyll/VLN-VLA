# -*- coding: utf-8 -*-
"""P0 部署脚本: 把 .stage_p0 里的改动推送到 VM 的 vla_ws/src + deploy 归档。"""
import io
import os
import posixpath

import paramiko

HOST, USER, PWD = '192.168.255.128', 'earl', '111111'
HOME = '/home/earl/workspace/VLA'

# 本地 -> 远端(相对 HOME)
MAP = {
    '.stage_p0/gt_tf.py': [
        'vla_ws/src/vla_follow/vla_follow/gt_tf.py',
        'deploy/gt_tf.py',
    ],
    '.stage_p0/cmd_mux.py': [
        'vla_ws/src/vla_follow/vla_follow/cmd_mux.py',
        'deploy/cmd_mux.py',
    ],
    '.stage_p0/follower.py': [
        'vla_ws/src/vla_follow/vla_follow/follower.py',
        'deploy/follower.py',
    ],
    '.stage_p0/nav2_params.yaml': [
        'vla_ws/src/vla_follow/config/nav2_params.yaml',
        'deploy/nav2_params.yaml',
    ],
    '.stage_p0/nav2_follow.launch.py': [
        'vla_ws/src/vla_follow/launch/nav2_follow.launch.py',
        'deploy/nav2_follow.launch.py',
    ],
    '.stage_p0/demo.launch.py': [
        'vla_ws/src/vla_follow/launch/demo.launch.py',
        'deploy/demo.launch.py',
    ],
}


def main():
    cli = paramiko.SSHClient()
    cli.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    cli.connect(HOST, 22, USER, PWD, timeout=10,
                look_for_keys=False, allow_agent=False)
    sftp = cli.open_sftp()

    def run(cmd, t=600, echo=True):
        _i, o, e = cli.exec_command(cmd, timeout=t)
        out = o.read().decode('utf-8', 'replace')
        err = e.read().decode('utf-8', 'replace')
        if echo:
            if out.strip():
                print(out.rstrip())
            if err.strip():
                print('[ERR]', err.rstrip()[:2000])
        return out, err

    base = os.path.dirname(os.path.abspath(__file__))
    n = 0
    for src, dests in MAP.items():
        lp = os.path.join(base, src)
        data = io.open(lp, 'rb').read()
        with io.open(lp, encoding='utf-8') as f:
            f.read()
        for d in dests:
            rp = posixpath.join(HOME, d)
            # 确保目录存在
            run('mkdir -p %s' % posixpath.dirname(rp), echo=False)
            with sftp.file(rp, 'wb') as fh:
                fh.write(data)
            n += 1
            print('upload %-52s %6d B' % (d, len(data)))
    sftp.close()

    print('\n=== 语法检查 ===')
    run('cd %s && python3 -m py_compile '
        'vla_ws/src/vla_follow/vla_follow/gt_tf.py '
        'vla_ws/src/vla_follow/vla_follow/cmd_mux.py '
        'vla_ws/src/vla_follow/launch/nav2_follow.launch.py '
        'vla_ws/src/vla_follow/launch/demo.launch.py '
        '&& echo PY_SYNTAX_OK' % HOME)

    print('\n=== P68: deploy -> src 回灌 (拿回 Nav2 版 follower 与 P20/P38 版感知) ===')
    run('cd %s && mkdir -p .bak_p0 && '
        'for f in follower person_vision person_walk; do '
        'cp -p vla_ws/src/vla_follow/vla_follow/$f.py .bak_p0/$f.py.src_old; '
        'cp -f deploy/$f.py vla_ws/src/vla_follow/vla_follow/$f.py; done; '
        'echo SYNC_DONE' % HOME)
    run('cd %s && for f in follower person_vision person_walk depth_percept '
        'gt_tf cmd_mux; do printf "%%-14s src=%%s deploy=%%s\\n" $f '
        '$(md5sum vla_ws/src/vla_follow/vla_follow/$f.py|cut -c1-8) '
        '$(md5sum deploy/$f.py 2>/dev/null|cut -c1-8); done' % HOME)

    print('\n=== yaml 解析检查 ===')
    run('python3 -c "import yaml,io;'
        'd=yaml.safe_load(io.open(\'%s/vla_ws/src/vla_follow/config/'
        'nav2_params.yaml\',encoding=\'utf-8\'));'
        'print(\'YAML_OK keys:\', list(d))"' % HOME)
    cli.close()
    print('\nDONE, %d files uploaded' % n)


if __name__ == '__main__':
    main()
