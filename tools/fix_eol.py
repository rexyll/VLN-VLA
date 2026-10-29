#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""fix_eol.py —— 把仓库里的文本文件行尾统一成 LF(消除 CRLF)。

为什么必须做这件事(代价: 又排查了一整轮):
    在 Windows 上编辑/生成的 .sh 会被写成 CRLF。部署到 Linux 后 bash 看到的
    是 `KEEP_GZ=0\\r`、`/dev/null\\r`、`sleep 2\\r`, 于是:
        line 19: $'\\r': command not found
        line 23: /dev/null: Permission denied
    脚本**从第一行就静默失效**, 里面的 pkill 一条都没执行过。
    表现却是"每次 start_scene 都多留一个 gz 世界", 越叠越多(实测到 19 个),
    订阅方读到多份交错的 dynamic_pose/info, 于是水壶被"掀飞 6m"、关节
    "偏差 160°" —— 全是假象。CR 是看不见的, 所以这个坑反复复发。

用法:
    python tools/fix_eol.py            # 只报告
    python tools/fix_eol.py --fix      # 就地转成 LF
"""
import os
import sys

# 只处理这些后缀; 二进制/模型文件不碰
TEXT_EXT = (
    ".sh", ".py", ".yaml", ".yml", ".md", ".txt", ".xml", ".sdf", ".urdf",
    ".xacro", ".cfg", ".json", ".csv", ".rviz", ".launch", ".service",
)
SKIP_DIRS = {"__pycache__", ".git", "build", "install", "log", "logs",
             ".workbuddy", ".pytest_cache", "node_modules"}


def scan(root="."):
    bad = []
    for dp, dns, fns in os.walk(root):
        dns[:] = [d for d in dns if d not in SKIP_DIRS]
        for fn in fns:
            if not fn.endswith(TEXT_EXT):
                continue
            p = os.path.join(dp, fn)
            try:
                with open(p, "rb") as f:
                    b = f.read()
            except OSError:
                continue
            if b"\r\n" in b:
                bad.append((p, b.count(b"\r\n")))
    return bad


def normalize(p):
    with open(p, "rb") as f:
        b = f.read()
    # 先统一 CRLF -> LF, 再清掉孤立 CR
    nb = b.replace(b"\r\n", b"\n").replace(b"\r", b"\n")
    if nb != b:
        with open(p, "wb") as f:
            f.write(nb)
    return len(b) - len(nb)


def main():
    fix = "--fix" in sys.argv
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    bad = scan(root)
    print("CRLF 文件数: %d" % len(bad))
    for p, n in sorted(bad, key=lambda x: -x[1]):
        rel = os.path.relpath(p, root).replace("\\", "/")
        if fix:
            saved = normalize(p)
            print("  FIX  %-58s %5d 处 (省 %d 字节)" % (rel, n, saved))
        else:
            print("  BAD  %-58s %5d 处" % (rel, n))
    if fix and bad:
        print("\n完成: %d 个文件已转为 LF" % len(bad))
    elif bad:
        print("\n加 --fix 就地修复")


if __name__ == "__main__":
    main()
