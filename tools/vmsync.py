#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""vmsync.py —— 本地 F:/workspace/_coding_workspace/VLA  <->  VM /home/earl/workspace/VLA
双向同步工具(SFTP)。

子命令:
  status              显示同步工具当前开关状态(不连VM)
  enable              打开同步工具开关(写 .vmsync_enabled 标记文件)
  disable             关闭同步工具开关(删除标记文件) —— 默认状态
  scan                列出两端清单并做差异比对(只读)
  pull <relpath>      从 VM 拉取文件到本地
  push <relpath>      把本地文件推到 VM
  apply               按比对结果执行同步(新文件双向传, 冲突按 mtime 新者胜)
  verify              同步后校验(逐文件 md5)

排除规则: __pycache__ / .git / vla_ws/build / vla_ws/install / vla_ws/log

开关机制:
  同步工具默认处于【关闭】状态(仓库根无 .vmsync_enabled 标记文件)。
  关闭状态下执行任何实际同步动作都会直接拒绝并提示, 且不会连接 VM。
  临时使用可用环境变量 VMSYNC_ENABLE=1 强制放行; 长期打开请执行:
      python vmsync.py enable
"""
import hashlib
import os
import posixpath
import stat as stat_mod
import sys
import time

import paramiko

HOST, PORT, USER, PW = "192.168.255.128", 22, "earl", "111111"
LOCAL_ROOT = os.environ.get("VLA_LOCAL_ROOT", r"Z:\VLA")
REMOTE_ROOT = os.environ.get("VLA_REMOTE_ROOT", "/home/earl/workspace/VLA")

# 同步工具开关标记文件: 存在 = 打开; 不存在 = 关闭(默认)
ENABLE_FLAG = os.path.join(LOCAL_ROOT, ".vmsync_enabled")
# 强制放行的环境变量名 (值非 "0"/""/unset 时生效)
ENABLE_ENV = "VMSYNC_ENABLE"

EXCLUDE_DIRS = {
    "__pycache__", ".git", ".pytest_cache", ".obsidian", ".zwork",
    "build", "install", "log",
}
# vla_ws 下的 build/install/log 由 colcon 生成, 不纳入同步
EXCLUDE_PATHS = {
    "vla_ws/build", "vla_ws/install", "vla_ws/log", "vla_ws/logs",
}

# VM 端 follower 正在实时追加的日志: 字节级必然持续变化,
# 只在同步最后一步统一"VM -> 本地"拉一次快照, 不参与差异冲突判定。
LIVE_LOGS = {
    "p60_run.log", "traj_fov.log", "traj_log.csv", "eval.json",
}


def excluded(rel):
    parts = rel.replace("\\", "/").split("/")
    if any(p in EXCLUDE_DIRS for p in parts[:-1]):
        return True
    rp = "/".join(parts)
    for ex in EXCLUDE_PATHS:
        if rp == ex or rp.startswith(ex + "/"):
            return True
    return False


def conn():
    c = paramiko.SSHClient()
    c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    c.connect(HOST, PORT, USER, PW, timeout=30,
              look_for_keys=False, allow_agent=False)
    return c


def md5_file(path):
    h = hashlib.md5()
    try:
        with open(path, "rb") as f:
            for blk in iter(lambda: f.read(1 << 20), b""):
                h.update(blk)
    except OSError:
        return None
    return h.hexdigest()


def local_list():
    out = {}
    for dp, dns, fns in os.walk(LOCAL_ROOT):
        dns[:] = [d for d in dns if d not in EXCLUDE_DIRS]
        for fn in fns:
            fp = os.path.join(dp, fn)
            rel = os.path.relpath(fp, LOCAL_ROOT).replace("\\", "/")
            if excluded(rel):
                continue
            try:
                st = os.stat(fp)
            except OSError:
                continue
            out[rel] = (st.st_size, int(st.st_mtime))
    return out


def remote_list(sftp):
    out = {}
    stack = [("", REMOTE_ROOT)]
    while stack:
        rel, rdir = stack.pop()
        try:
            entries = sftp.listdir_attr(rdir)
        except OSError:
            continue
        for e in entries:
            # 隐藏目录默认跳过, 但 .workbuddy 是项目工作台(含 memory 日志), 必须纳入
            if e.filename.startswith(".") and e.filename != ".workbuddy":
                continue
            rp = posixpath.join(rdir, e.filename)
            rrel = posixpath.join(rel, e.filename) if rel else e.filename
            if stat_mod.S_ISDIR(e.st_mode):
                if e.filename in EXCLUDE_DIRS:
                    continue
                if excluded(rrel):
                    continue
                stack.append((rrel, rp))
            else:
                if excluded(rrel):
                    continue
                out[rrel] = (e.st_size, int(e.st_mtime))
    return out


def remote_md5(sftp, rp):
    try:
        h = hashlib.md5()
        with sftp.open(rp, "rb") as f:
            for blk in iter(lambda: f.read(1 << 20), b""):
                h.update(blk)
        return h.hexdigest()
    except OSError:
        return None


def cmd_scan(c, sftp):
    L = local_list()
    R = remote_list(sftp)
    lk, rk = set(L), set(R)
    only_l = sorted(lk - rk)
    only_r = sorted(rk - lk)
    both = sorted(lk & rk)
    diff_size = [k for k in both if L[k][0] != R[k][0]]
    print("=" * 68)
    print("本地文件 %d 个 | VM 文件 %d 个 | 共有 %d 个" % (len(L), len(R), len(both)))
    print("=" * 68)
    print("\n[仅本地有] %d 个:" % len(only_l))
    for k in only_l[:400]:
        print("  L  %-70s %10d" % (k, L[k][0]))
    if len(only_l) > 400:
        print("  ... 另有 %d 个" % (len(only_l) - 400))
    print("\n[仅VM有] %d 个:" % len(only_r))
    for k in only_r[:400]:
        print("  R  %-70s %10d" % (k, R[k][0]))
    if len(only_r) > 400:
        print("  ... 另有 %d 个" % (len(only_r) - 400))
    print("\n[共有但大小不同] %d 个:" % len(diff_size))
    for k in diff_size[:200]:
        print("  ?  %-60s local=%-10d vm=%-10d" % (k, L[k][0], R[k][0]))
    # 大小相同但内容可能不同 -> 抽样 md5
    same_size = [k for k in both if L[k][0] == R[k][0]]
    print("\n[共有且大小相同] %d 个 (内容需 md5 校验)" % len(same_size))
    return L, R, only_l, only_r, diff_size, same_size


def cmd_apply(c, sftp, direction="both"):
    L = local_list()
    R = remote_list(sftp)
    only_l = sorted(set(L) - set(R))
    only_r = sorted(set(R) - set(L))
    both = sorted(set(L) & set(R))

    # 冲突判定: 大小不同, 或大小相同但 md5 不同
    conflicts = []
    for k in both:
        if L[k][0] != R[k][0]:
            conflicts.append(k)
            continue
        rm = remote_md5(sftp, posixpath.join(REMOTE_ROOT, k))
        lm = md5_file(os.path.join(LOCAL_ROOT, k.replace("/", os.sep)))
        if rm != lm:
            conflicts.append(k)

    print("冲突文件 %d 个, 逐个按 mtime 新者胜:" % len(conflicts))
    n_push = n_pull = 0
    for k in conflicts:
        lm_time, rm_time = L[k][1], R[k][1]
        if lm_time > rm_time + 1:
            push_file(sftp, k, c)
            n_push += 1
            print("  PUSH(本地新) %s" % k)
        else:
            pull_file(sftp, k)
            n_pull += 1
            print("  PULL(VM 新)   %s" % k)
    print("\n冲突处理: push=%d pull=%d" % (n_push, n_pull))
    return conflicts


def _mkdirs(sftp, rdir):
    parts = rdir.strip("/").split("/")
    cur = ""
    for p in parts:
        cur = "/" + p if not cur else cur + "/" + p
        try:
            sftp.stat(cur)
        except OSError:
            try:
                sftp.mkdir(cur)
            except OSError:
                pass


# 上传时必须归一化行尾的文本后缀。
# 根因: Windows 侧编辑会把 .sh 写成 CRLF, 部署到 Linux 后 bash 读到
#       `KEEP_GZ=0\r` / `/dev/null\r` / `sleep 2\r`, 脚本从第一行就静默失效。
#       killall_demo.sh 因此里面的 pkill 一条都没跑, 每次 start_scene 都
#       多留一个 gz 世界, 越叠越多(实测 19 个) —— 症状却是"水壶被掀飞 6m"
#       这种物理假象, 极难联想到 CR。
TEXT_EXT = (
    ".sh", ".py", ".yaml", ".yml", ".md", ".txt", ".xml", ".sdf", ".urdf",
    ".xacro", ".cfg", ".json", ".csv", ".rviz", ".launch", ".service",
)


def normalize_eol_bytes(b):
    """CRLF -> LF。仅为文本文件调用。"""
    return b.replace(b"\r\n", b"\n").replace(b"\r", b"\n")


def push_file(sftp, rel, client=None):
    """上传本地文件到 VM。

    该服务器的 SFTP rename 覆盖会失败(OSError: Failure), 所以统一
    "先传同目录隐藏临时文件, 再用 exec_command 执行 mv 原子替换"。
    文本文件先做 CRLF -> LF 归一化(见 TEXT_EXT 上的注释)。
    """
    lfp = os.path.join(LOCAL_ROOT, rel.replace("/", os.sep))
    rfp = posixpath.join(REMOTE_ROOT, rel)
    rdir = posixpath.dirname(rfp)
    _mkdirs(sftp, rdir)
    tmp = rdir + "/.up_" + posixpath.basename(rfp) + ".tmp"
    try:
        sftp.remove(tmp)
    except OSError:
        pass
    if rel.lower().endswith(TEXT_EXT):
        with open(lfp, "rb") as f:
            data = normalize_eol_bytes(f.read())
        with sftp.open(tmp, "wb") as f:
            f.write(data)
    else:
        sftp.put(lfp, tmp)
    if client is None:
        raise RuntimeError("push_file 需要 client 才能执行 mv 原子替换")
    cmd = "mv -f '%s' '%s' && wc -c '%s'" % (tmp, rfp, rfp)
    _in, out, err = client.exec_command(cmd, timeout=120)
    rc = out.channel.recv_exit_status()
    if rc != 0:
        raise RuntimeError("mv 失败 rc=%d %s" % (rc, err.read().decode("utf-8", "replace")))


def pull_file(sftp, rel):
    rfp = posixpath.join(REMOTE_ROOT, rel)
    lfp = os.path.join(LOCAL_ROOT, rel.replace("/", os.sep))
    _mkdirs_local(os.path.dirname(lfp))
    tmp = lfp + ".dl_tmp"
    sftp.get(rfp, tmp)
    if os.path.exists(lfp):
        os.remove(lfp)
    os.rename(tmp, lfp)


def _mkdirs_local(d):
    if d and not os.path.isdir(d):
        os.makedirs(d, exist_ok=True)


def cmd_sync(c, sftp):
    """完整双向同步:
       1) 仅本地有的 -> 推送到 VM
       2) 仅 VM 有的   -> 拉回本地
       3) 共有但内容不同 -> 按 mtime 新者胜(实时日志除外)
       4) 实时日志      -> 最后统一 VM -> 本地 拉一次快照
    """
    L = local_list()
    R = remote_list(sftp)
    only_l = sorted(set(L) - set(R))
    only_r = sorted(set(R) - set(L))
    both = sorted(set(L) & set(R))

    print("[1/4] 推送仅本地有的文件: %d 个" % len(only_l))
    for rel in only_l:
        push_file(sftp, rel, c)
    print("      完成")

    print("[2/4] 拉取仅VM有的文件: %d 个" % len(only_r))
    for rel in only_r:
        pull_file(sftp, rel)
    print("      完成")

    print("[3/4] 处理共有但内容不同的文件")
    conflicts = []
    for k in both:
        if k in LIVE_LOGS:
            continue
        if L[k][0] != R[k][0]:
            conflicts.append(k)
            continue
        rm = remote_md5(sftp, posixpath.join(REMOTE_ROOT, k))
        lm = md5_file(os.path.join(LOCAL_ROOT, k.replace("/", os.sep)))
        if rm != lm:
            conflicts.append(k)
    npush = npull = 0
    for k in conflicts:
        if L[k][1] > R[k][1] + 1:
            push_file(sftp, k, c); npush += 1
        else:
            pull_file(sftp, k); npull += 1
    print("      冲突 %d 个 -> push=%d pull=%d" % (len(conflicts), npush, npull))

    print("[4/4] 实时日志统一 VM->本地 拉快照")
    live = [k for k in sorted(set(L) & set(R)) if k in LIVE_LOGS]
    for rel in live:
        pull_file(sftp, rel)
    print("      已刷新 %d 个: %s" % (len(live), ", ".join(live)))
    return conflicts


def sync_enabled():
    """同步工具开关状态。

    返回 True 表示允许执行实际同步动作。
    - 标记文件 .vmsync_enabled 存在  -> 打开
    - 环境变量 VMSYNC_ENABLE 非空且非 "0" -> 临时强制放行
    """
    if os.path.exists(ENABLE_FLAG):
        return True
    env = os.environ.get(ENABLE_ENV, "").strip()
    return bool(env) and env != "0"


def cmd_status():
    on = sync_enabled()
    src = "标记文件 %s" % ENABLE_FLAG if os.path.exists(ENABLE_FLAG) else \
          ("环境变量 %s=%s" % (ENABLE_ENV, os.environ.get(ENABLE_ENV, "")) if sync_enabled() else "默认")
    print("=" * 66)
    print("同步工具 vmsync: %s" % ("【打开】ON" if on else "【关闭】OFF"))
    print("判定来源: %s" % src)
    print("本地根: %s" % LOCAL_ROOT)
    print("远端根: %s" % REMOTE_ROOT)
    print("=" * 66)
    return on


def cmd_enable():
    with open(ENABLE_FLAG, "w", encoding="utf-8") as f:
        f.write("vmsync enabled at %s\n" % time.strftime("%Y-%m-%d %H:%M:%S"))
    print("[ok] 同步工具已打开 (标记文件: %s)" % ENABLE_FLAG)
    print("     执行: python vmsync.py scan|sync|verify")


def cmd_disable():
    if os.path.exists(ENABLE_FLAG):
        os.remove(ENABLE_FLAG)
        print("[ok] 同步工具已关闭 (已删除标记文件)")
    else:
        print("[ok] 同步工具本来就是关闭状态")


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else "scan"

    # ---- 开关闸门: status/enable/disable 始终可用, 其余需先打开 ----
    if mode == "status":
        cmd_status()
        return
    if mode == "enable":
        cmd_enable()
        return
    if mode == "disable":
        cmd_disable()
        return
    if not sync_enabled():
        print("=" * 66)
        print("同步工具当前处于【关闭】状态, 已拒绝执行 '%s'。" % mode)
        print("工具不会连接 VM, 不会读取/写入任何文件。")
        print("如需使用, 请执行:  python vmsync.py enable")
        print("        或单次放行:  VMSYNC_ENABLE=1 python vmsync.py %s" % mode)
        print("=" * 66)
        sys.exit(2)

    c = conn()
    sftp = c.open_sftp()
    try:
        if mode == "scan":
            cmd_scan(c, sftp)
        elif mode == "sync":
            cmd_sync(c, sftp)
        elif mode == "apply":
            cmd_apply(c, sftp)
        elif mode == "pull":
            for rel in sys.argv[2:]:
                pull_file(sftp, rel)
                print("PULL ok %s" % rel)
        elif mode == "push":
            for rel in sys.argv[2:]:
                push_file(sftp, rel, c)
                print("PUSH ok %s" % rel)
        elif mode == "verify":
            L = local_list()
            R = remote_list(sftp)
            bad = []
            live = []
            for k in sorted(set(L) & set(R)):
                rm = remote_md5(sftp, posixpath.join(REMOTE_ROOT, k))
                lm = md5_file(os.path.join(LOCAL_ROOT, k.replace("/", os.sep)))
                if rm != lm:
                    if k in LIVE_LOGS:
                        live.append(k)
                    else:
                        bad.append(k)
            only_l = sorted(set(L) - set(R))
            only_r = sorted(set(R) - set(L))
            print("=" * 70)
            print("共有 %d | md5 不一致 %d | 仅本地 %d | 仅VM %d"
                  % (len(set(L) & set(R)), len(bad), len(only_l), len(only_r)))
            print("实时日志(持续追加, 不计为不一致): %s" % (", ".join(live) or "无"))
            for k in only_l:
                print("  L-only %s" % k)
            for k in only_r:
                print("  R-only %s" % k)
            for k in bad:
                print("  MISMATCH %s" % k)
            print("=" * 70)
            print("结果: 两端完全一致" if not (only_l or only_r or bad)
                  else "结果: 仍有 %d 处差异" % (len(only_l) + len(only_r) + len(bad)))
        else:
            print("unknown mode")
    finally:
        c.close()


if __name__ == "__main__":
    main()
