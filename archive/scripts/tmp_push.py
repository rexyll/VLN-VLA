#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tmp_push.py —— 用相对路径把目录推到 VM 家目录(绕开 SFTP 绝对路径怪癖)
用法: py tmp_push.py <本地目录> <远程相对目录>
"""
import os
import sys

import paramiko

HOST, PORT, USER, PW = "192.168.255.128", 22, "earl", "111111"
SKIP_DIRS = {"__pycache__", ".git", "runs"}
SKIP_EXT = {".pyc"}


def main():
    lroot, rroot = sys.argv[1], sys.argv[2].replace("\\", "/").strip("/")
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(HOST, PORT, USER, PW, timeout=15,
              look_for_keys=False, allow_agent=False)
    sftp = c.open_sftp()
    n = 0
    for root, dirs, files in os.walk(lroot):
        dirs[:] = [d for d in dirs if d not in SKIP_DIRS]
        rel = os.path.relpath(root, lroot).replace("\\", "/")
        rdir = rroot if rel == "." else rroot + "/" + rel
        for f in files:
            if os.path.splitext(f)[1].lower() in SKIP_EXT:
                continue
            sftp.put(os.path.join(root, f), rdir + "/" + f)
            n += 1
            print("PUT", rdir + "/" + f)
    print(f"PUSH OK {n} files -> {rroot}")
    c.close()


if __name__ == "__main__":
    main()
