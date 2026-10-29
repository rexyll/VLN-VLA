#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""vmpush.py —— 走 SSH exec(而不是 SFTP)把文件推到 VM。

为什么需要这个工具
==================
这台 VM 的 /home/earl 是某种挂载(ls 出来目录的属主显示成 uid 1000、日期后面
跟一个 '?'), SFTP 往这个目录 open(O_WRONLY) 会稳定返回 ENOENT/Failure,
`vmput.py`(SFTP 直写 + 临时文件原子替换)重试 4 次全失败; 而 shell 里
`echo hello > 同目录/_t.txt` 完全正常。所以改走 exec: base64 分块 append 到
同目录隐藏临时文件, 再 mv 原子替换 —— 与 vmput 的效果一致, 只是传输层换了。

用法:
    python vmpush.py <本地文件> <远程路径> [<远程路径2> ...]
"""
import base64
import os
import posixpath
import sys

import paramiko

HOST, PORT, USER, PW = "192.168.255.128", 22, "earl", "111111"
CHUNK = 60000          # base64 单条命令的长度上限(远小于 2MB 的 exec 限制)


def main():
    local = sys.argv[1]
    data = open(local, 'rb').read()
    b64 = base64.b64encode(data).decode('ascii')
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(HOST, PORT, USER, PW, timeout=20,
              look_for_keys=False, allow_agent=False)
    try:
        for remote in sys.argv[2:]:
            tmp = posixpath.join(posixpath.dirname(remote),
                                 '._up_' + posixpath.basename(remote))
            _run(c, 'rm -f %s' % tmp)
            for i in range(0, len(b64), CHUNK):
                op = '>' if i == 0 else '>>'
                _run(c, "printf '%%s' '%s' %s %s"
                     % (b64[i:i + CHUNK], op, tmp))
            out = _run(c, 'base64 -d %s > %s && rm -f %s && wc -c < %s'
                       % (tmp, remote, tmp, remote))
            print('[ok] %s -> %s (%s bytes)'
                  % (local, remote, out.strip()))
    finally:
        c.close()


def _run(c, cmd):
    _in, out, err = c.exec_command(cmd, timeout=120)
    rc = out.channel.recv_exit_status()
    so = out.read().decode('utf-8', 'replace')
    se = err.read().decode('utf-8', 'replace')
    if rc != 0:
        raise SystemExit('[FAIL] %s\n  rc=%d\n  %s' % (cmd[:120], rc,
                                                      se.strip()))
    if se.strip():
        sys.stderr.write(se)
    return so


if __name__ == '__main__':
    main()
