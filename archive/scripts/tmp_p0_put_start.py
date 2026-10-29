# -*- coding: utf-8 -*-
"""上传 vla_demo 下的两个 shell 脚本并赋可执行权限"""
import io
import os
import posixpath

import paramiko

HOST, USER, PWD = '192.168.255.128', 'earl', '111111'
HOME = '/home/earl/workspace/VLA'
FILES = {
    '.stage_p0/start_demo.sh': 'vla_demo/start_demo.sh',
}


def main():
    cli = paramiko.SSHClient()
    cli.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    cli.connect(HOST, 22, USER, PWD, timeout=10,
                look_for_keys=False, allow_agent=False)
    sftp = cli.open_sftp()
    base = os.path.dirname(os.path.abspath(__file__))
    for src, dst in FILES.items():
        data = io.open(os.path.join(base, src), 'rb').read()
        rp = posixpath.join(HOME, dst)
        with sftp.file(rp, 'wb') as fh:
            fh.write(data)
        sftp.chmod(rp, 0o755)
        print('upload %-30s %6d B (chmod 755)' % (dst, len(data)))
    sftp.close()
    _i, o, e = cli.exec_command(
        'bash -n %s/vla_demo/start_demo.sh && echo SYNTAX_OK; '
        'grep -c "" %s/vla_demo/start_demo.sh' % (HOME, HOME), timeout=30)
    print(o.read().decode('utf-8', 'replace'))
    print('[ERR]', e.read().decode('utf-8', 'replace')[:300])
    cli.close()


if __name__ == '__main__':
    main()
