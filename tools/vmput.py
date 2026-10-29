#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""vmput.py —— 基于 SFTP 的文件上传助手(带临时文件+原子替换与重试)。

SFTP 直接写目标路径在某些目录会偶发 ENOENT, 所以统一先传到同目录下的
隐藏临时文件, 再用 mv 原子替换。

用法:
  python vmput.py <本地文件> <远程路径> [<远程路径2> ...]
"""
import posixpath
import sys
import time

import paramiko

HOST, PORT, USER, PW = "192.168.255.128", 22, "earl", "111111"


def _put_once(sftp, local, remote):
    tmp = posixpath.join(posixpath.dirname(remote),
                         '.up_' + posixpath.basename(remote) + '.tmp')
    sftp.put(local, tmp)
    sftp.rename(tmp, remote)
    return sftp.stat(remote).st_size


def main():
    local = sys.argv[1]
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(HOST, PORT, USER, PW, timeout=20,
              look_for_keys=False, allow_agent=False)
    try:
        sftp = c.open_sftp()
        for remote in sys.argv[2:]:
            last = None
            for attempt in range(4):
                try:
                    size = _put_once(sftp, local, remote)
                    print('[ok] %s -> %s (%d bytes)'
                          % (local, remote, size))
                    last = None
                    break
                except Exception as e:      # noqa: BLE001
                    last = e
                    time.sleep(1.0 + attempt)
            if last is not None:
                raise SystemExit('[FAIL] %s -> %s : %s'
                                 % (local, remote, last))
        sftp.close()
    finally:
        c.close()


if __name__ == "__main__":
    main()
