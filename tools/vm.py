#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""vm.py —— SSH 助手(sftp 不可用时用 base64 传输)

用法:
  python vm.py exec "命令" [超时]
  python vm.py put 本地文件 远程路径
  python vm.py get 远程路径 本地文件
  python vm.py sudo "命令"        # 免密或回显密码

注意(2026-10-26): 在 Git Bash(MSYS2) 里直接调用本脚本时, 以 `/` 开头的
远程路径参数会被 MSYS2 当成 Unix 路径自动改写成 Windows 路径, 例如
    /home/earl/x  ->  C:/Program Files/Git/home/earl/x
远程 bash 于是收到 `C:/Program` 当命令, 报
    bash: line 1: C:/Program: No such file or directory
规避: 调用前 export MSYS2_ARG_CONV_EXCL='*'
本脚本另加了 _remote() 兜底还原, 两条路都通。
"""
import base64
import sys

import paramiko

HOST, PORT, USER, PW = "192.168.255.128", 22, "earl", "111111"

# 上传时归一化行尾的文本后缀(CRLF -> LF)
TEXT_EXT = (
    ".sh", ".py", ".yaml", ".yml", ".md", ".txt", ".xml", ".sdf", ".urdf",
    ".xacro", ".cfg", ".json", ".csv", ".rviz", ".launch", ".service",
)

# MSYS2/GitBash 误改写远程路径时留下的前缀, 逐个剥掉还原
_MSYS_PREFIX = (
    "C:/Program Files/Git",
    "C:/Program Files (x86)/Git",
    "C:/msys64",
    "C:/cygwin64",
)


def _remote(arg):
    """把被 MSYS2 改写过的远程路径还原成 / 开头的 POSIX 路径。"""
    a = arg
    for pre in _MSYS_PREFIX:
        if a.startswith(pre):
            a = a[len(pre):]
            break
    return a


def client():
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(HOST, PORT, USER, PW, timeout=20,
              look_for_keys=False, allow_agent=False)
    return c


def run(c, cmd, timeout=600):
    _in, out, err = c.exec_command(cmd, timeout=timeout)
    data = out.read()
    rc = out.channel.recv_exit_status()
    text = data.decode("utf-8", "replace")
    e = err.read().decode("utf-8", "replace")
    sys.stdout.write(text)
    if e.strip():
        sys.stdout.write("\n[STDERR]\n" + e[-3000:])
    sys.stdout.write(f"\n[RC={rc}]\n")
    return rc


def main():
    mode = sys.argv[1]
    c = client()
    try:
        if mode == "exec":
            run(c, sys.argv[2], int(sys.argv[3]) if len(sys.argv) > 3 else 600)
        elif mode == "put":
            raw = open(sys.argv[2], "rb").read()
            # 文本文件归一化 CRLF -> LF。
            # 根因: 在 Windows 上编辑的 .sh 带 CRLF, 部署到 Linux 后 bash 读到
            #       `KEEP_GZ=0\r` / `/dev/null\r`, 脚本从第一行就静默失效 ——
            #       killall_demo.sh 里的 pkill 一次都没执行, 于是每次启动都多留
            #       一个 gz 世界, 叠加到 19 个后所有位姿数据都是多份交错值,
            #       表现为"水壶被掀飞 6m / 关节偏 160°"这类纯物理假象。
            if sys.argv[2].lower().endswith(TEXT_EXT):
                raw = raw.replace(b"\r\n", b"\n").replace(b"\r", b"\n")
            b64 = base64.b64encode(raw).decode()
            remote = _remote(sys.argv[3])
            chunk = 3000
            run(c, f": > {remote}.b64")
            for i in range(0, len(b64), chunk):
                run(c, f"printf '%s' '{b64[i:i+chunk]}' >> {remote}.b64")
            run(c, f"base64 -d {remote}.b64 > {remote} && rm {remote}.b64 "
                   f"&& wc -c {remote}")
        elif mode == "get":
            run(c, f"base64 -w0 {_remote(sys.argv[2])}")
        else:
            raise SystemExit("unknown mode")
    finally:
        c.close()


if __name__ == "__main__":
    main()
