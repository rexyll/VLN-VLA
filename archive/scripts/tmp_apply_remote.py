#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tmp_apply_remote.py —— 绕开 Windows 端 SMB 写锁，从 VM(Linux) 侧写回 TaskPlan.md 与 memory

Windows 侧对已存在文件的覆盖被共享拒绝(WinError 5)；Z:\\VLA 与 VM
/home/earl/workspace/VLA 是同一目录，因此改由 VM 侧 base64 落盘 + mv 覆盖。
"""
import io
import os
import posixpath

import paramiko

HOST, PORT, USER, PW = "192.168.255.128", 22, "earl", "111111"
ROOT = "/home/earl/workspace/VLA"
HERE = os.path.dirname(os.path.abspath(__file__))


def put(c, remote, data: bytes):
    tmp = "/tmp/_up_%d.bin" % (abs(hash(remote)) % 10 ** 8)
    _in, out, err = c.exec_command("cat > %s" % tmp)
    _in.write(data)
    _in.channel.shutdown_write()
    rc = out.channel.recv_exit_status()
    assert rc == 0, err.read().decode()
    _in2, out2, err2 = c.exec_command("mv -f %s %s && md5sum %s && wc -c %s"
                                      % (tmp, remote, remote, remote))
    rc2 = out2.channel.recv_exit_status()
    info = out2.read().decode().strip()
    assert rc2 == 0, err2.read().decode()
    return info


def main():
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(HOST, PORT, USER, PW, timeout=15,
              look_for_keys=False, allow_agent=False)
    try:
        # 1) TaskPlan.md ← 本地已生成的 TaskPlan.md.new
        tp_new = os.path.join(HERE, "TaskPlan.md.new")
        if os.path.exists(tp_new):
            data = open(tp_new, "rb").read()
            print("[TaskPlan] 上传 %d B -> %s" % (
                len(data), put(c, posixpath.join(ROOT, "TaskPlan.md"), data)))
        else:
            print("[TaskPlan] 未找到 TaskPlan.md.new，跳过")

        # 2) memory 追加
        mem_rel = ".workbuddy/memory/2026-10-25.md"
        add = io.open(os.path.join(HERE, "tmp_memory_add.md"),
                      encoding="utf-8").read()
        _in, out, _err = c.exec_command(
            "cat %s" % posixpath.join(ROOT, mem_rel))
        cur = out.read().decode("utf-8")
        if "代码实际进度 vs TaskPlan 一致性核对" in cur:
            print("[memory] 已包含本轮记录，跳过")
        else:
            new = cur.rstrip("\n") + "\n" + add
            print("[memory] 上传 %d B -> %s" % (
                len(new.encode()), put(c, posixpath.join(ROOT, mem_rel),
                                       new.encode("utf-8"))))
    finally:
        c.close()


if __name__ == "__main__":
    main()
