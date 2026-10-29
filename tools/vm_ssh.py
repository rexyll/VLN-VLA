#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""vm_ssh.py —— 部署用 SSH 助手（会话工具，非交付代码）

用法:
    python vm_ssh.py exec "命令" [超时秒]
    python vm_ssh.py put 本地 远程
    python vm_ssh.py putdir 本地目录 远程目录     # 跳过 __pycache__/runs/*.npz/*.zip
    python vm_ssh.py get 远程 本地
"""
import os
import sys

import paramiko

HOST, PORT, USER, PW = "192.168.255.128", 22, "earl", "111111"
SKIP_DIRS = {"__pycache__", "runs", ".git"}
SKIP_EXT = {".pyc", ".npz", ".zip", ".pt"}


def client():
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(HOST, PORT, USER, PW, timeout=15,
              look_for_keys=False, allow_agent=False)
    return c


def main():
    mode = sys.argv[1]
    c = client()
    try:
        if mode == "exec":
            cmd = sys.argv[2]
            timeout = int(sys.argv[3]) if len(sys.argv) > 3 else 600
            _stdin, stdout, stderr = c.exec_command(cmd, timeout=timeout)
            rc = stdout.channel.recv_exit_status()
            sys.stdout.write(stdout.read().decode("utf-8", "replace"))
            err = stderr.read().decode("utf-8", "replace")
            if err.strip():
                sys.stdout.write("\n[STDERR]\n" + err[-4000:])
            sys.stdout.write(f"\n[RC={rc}]\n")
        elif mode == "put":
            sftp = c.open_sftp()
            sftp.put(sys.argv[2], sys.argv[3])
            print("PUT OK ->", sys.argv[3])
        elif mode == "putdir":
            sftp = c.open_sftp()
            lroot, rroot = sys.argv[2], sys.argv[3].replace("\\", "/")

            def mkdirs(rd):
                parts = [p for p in rd.split("/") if p]
                cur = ""
                for p in parts:
                    cur += "/" + p
                    try:
                        sftp.stat(cur)
                    except IOError:
                        sftp.mkdir(cur)
            mkdirs(rroot)
            n = 0
            for root, dirs, files in os.walk(lroot):
                dirs[:] = [d for d in dirs if d not in SKIP_DIRS]
                rel = os.path.relpath(root, lroot).replace("\\", "/")
                rdir = rroot if rel == "." else rroot + "/" + rel
                if rel != ".":
                    mkdirs(rdir)
                for f in files:
                    if os.path.splitext(f)[1].lower() in SKIP_EXT:
                        continue
                    sftp.put(os.path.join(root, f), rdir + "/" + f)
                    n += 1
            print(f"PUTDIR OK {n} files -> {rroot}")
        elif mode == "get":
            sftp = c.open_sftp()
            sftp.get(sys.argv[2], sys.argv[3])
            print("GET OK ->", sys.argv[3])
        else:
            print("unknown mode")
    finally:
        c.close()


if __name__ == "__main__":
    main()
