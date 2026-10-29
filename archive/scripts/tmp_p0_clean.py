# -*- coding: utf-8 -*-
"""彻底清场: 杀掉 demo 全部进程(gz/launch/nav2/业务节点/桥接) 并释放互斥锁。

单独写成脚本再执行, 是为了让 shell 自己的命令行里不含那些匹配模式
(否则 pkill -f 会匹配到自身, 把执行清理的这个 shell 一起杀掉 ——
 之前 P0 第一次清理失败、留下孤儿 Nav2 节点导致 bringup 随机 Abort 就是它)。
"""
import posixpath

import paramiko

HOST, USER, PWD = '192.168.255.128', 'earl', '111111'
HOME = '/home/earl/workspace/VLA'

CLEAN = r'''#!/bin/bash
# clean_all.sh —— 一键清场 (P71)
# 注意: 必须用 `pkill -f <完整路径前缀>` 而不是 `pkill -x <名字>` ——
#       Linux 的 comm 只有 15 字符, velocity_smoother/controller_server/
#       lifecycle_manager 会被截断成 velocity_smooth / controller_serv /
#       lifecycle_manag, 按全名精确匹配一个都杀不掉, 于是变成孤儿进程。
#       本脚本自己的命令行走的是 `bash clean_all.sh`, 不含下面任何模式,
#       所以 -f 不会自杀(在交互 shell 里直接敲才会, 故单独成脚本)。
echo ">>> 停 demo 进程"
pkill -f 'ros2 launch vla_follow' 2>/dev/null
pkill -f 'vla_follow/lib/vla_follow' 2>/dev/null
pkill -f parameter_bridge 2>/dev/null
pkill -f '/opt/ros/jazzy/lib/nav2_' 2>/dev/null
pkill -f 'gz sim' 2>/dev/null
pkill -f 'gz-sim' 2>/dev/null
sleep 2
pkill -9 -f '/opt/ros/jazzy/lib/nav2_' 2>/dev/null
pkill -9 -f 'gz sim' 2>/dev/null
pkill -9 -f 'gz-sim' 2>/dev/null
pkill -9 -f 'vla_follow/lib/vla_follow' 2>/dev/null
pkill -9 -f parameter_bridge 2>/dev/null
sleep 1
rm -f /tmp/vla_start_demo.lock
left=$(ps -eo args --no-headers | grep -E '/opt/ros/jazzy/lib/nav2_|gz sim|gz-sim|vla_follow/lib/vla_follow|parameter_bridge' | grep -v grep | wc -l)
echo ">>> 清场完成, 残留进程数: ${left}"
'''


def main():
    cli = paramiko.SSHClient()
    cli.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    cli.connect(HOST, 22, USER, PWD, timeout=10,
                look_for_keys=False, allow_agent=False)
    sftp = cli.open_sftp()

    # 1. clean_all.sh
    p = posixpath.join(HOME, 'vla_demo/clean_all.sh')
    with sftp.file(p, 'w') as f:
        f.write(CLEAN.encode('utf-8'))
    sftp.chmod(p, 0o755)
    print('写入 vla_demo/clean_all.sh')

    # 2. start_demo.sh: 清场改用路径前缀匹配 + 后台子进程不继承互斥锁 fd
    q = posixpath.join(HOME, 'vla_demo/start_demo.sh')
    with sftp.file(q, 'r') as f:
        txt = f.read().decode('utf-8')
    marker = '# P71: Nav2 节点清不掉'
    if marker in txt:
        i0 = txt.index(marker)
        i1 = txt.index('\ndone\n', i0) + len('\ndone\n')
        new_block = (
            '# P71: Nav2 节点清不掉会变孤儿 → 与下一轮同名共存, lifecycle 的\n'
            '#      服务调用打到旧实例上, 表现为随机的 "get_state service is\n'
            '#      not available" / "Aborting bringup"。\n'
            "#      必须按完整路径前缀杀: comm 只有 15 字符, velocity_smoother /\n"
            '#      controller_server / lifecycle_manager 会被截断, -x 全名匹配不到。\n'
            "pkill -f '/opt/ros/jazzy/lib/nav2_' 2>/dev/null\n")
        txt = txt[:i0] + new_block + txt[i1:]
        print('start_demo.sh: Nav2 清理改用路径前缀匹配')
    else:
        print('start_demo.sh 无 Nav2 清理块, 跳过')
    if '9>&-' in txt:
        print('start_demo.sh 已处理 fd9, 跳过')
    else:
        n = 0
        for old, new in (
            ("    > \"$VLA_HOME/vla_ws/logs/gzsim.log\" 2>&1 < /dev/null &",
             "    > \"$VLA_HOME/vla_ws/logs/gzsim.log\" 2>&1 < /dev/null 9>&- &"),
            ("    > \"$VLA_HOME/vla_ws/logs/demo.log\" 2>&1 < /dev/null &",
             "    > \"$VLA_HOME/vla_ws/logs/demo.log\" 2>&1 < /dev/null 9>&- &"),
        ):
            if old in txt:
                txt = txt.replace(old, new, 1)
                n += 1
        print('start_demo.sh 后台子进程已加 9>&- (%d 处): 互斥锁只防并发启动, '
              '不再被长跑的子进程长期占用(否则想重启必须手动清场)' % n)
    with sftp.file(q, 'w') as f:
        f.write(txt.encode('utf-8'))
    sftp.close()

    _i, o, e = cli.exec_command('bash %s/vla_demo/clean_all.sh' % HOME,
                                timeout=120)
    print(o.read().decode('utf-8', 'replace'))
    print('[ERR]', e.read().decode('utf-8', 'replace')[:300])
    cli.close()


if __name__ == '__main__':
    main()
