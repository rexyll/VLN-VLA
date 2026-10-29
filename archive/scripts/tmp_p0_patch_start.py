# -*- coding: utf-8 -*-
"""给 start_demo.sh 补上 Nav2 节点清理(精确进程名, 不用 -f 以免自匹配)"""
import posixpath

import paramiko

HOST, USER, PWD = '192.168.255.128', 'earl', '111111'
HOME = '/home/earl/workspace/VLA'

NAV2_PROCS = ('bt_navigator planner_server controller_server behavior_server '
              'velocity_smoother map_server collision_monitor docking_server '
              'waypoint_follower smoother_server route_server '
              'lifecycle_manager')

PATCH = """
# P71: Nav2 节点清不掉会变孤儿 → 与下一轮同名共存, lifecycle 服务调用打到
#      旧实例上, 表现为随机的 "get_state service is not available / Aborting
#      bringup"。这里按"精确进程名"补杀(-x), 不用 -f 以免匹配到本脚本自己。
for _n in __NAV2__; do
  pkill -x "$_n" 2>/dev/null
done
"""


def main():
    cli = paramiko.SSHClient()
    cli.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    cli.connect(HOST, 22, USER, PWD, timeout=10,
                look_for_keys=False, allow_agent=False)
    sftp = cli.open_sftp()
    p = posixpath.join(HOME, 'vla_demo/start_demo.sh')
    with sftp.file(p, 'r') as f:
        txt = f.read().decode('utf-8')

    if 'P71: Nav2 节点清不掉' in txt:
        print('已补丁过, 跳过')
    else:
        block = PATCH.replace('__NAV2__', NAV2_PROCS)
        anchor = "pkill -9 -f parameter_bridge 2>/dev/null\n"
        if anchor not in txt:
            raise SystemExit('找不到锚点, 放弃')
        txt = txt.replace(anchor, anchor + block, 1)
        with sftp.file(p, 'w') as f:
            f.write(txt.encode('utf-8'))
        print('start_demo.sh 已补 Nav2 进程清理')
    sftp.close()

    _i, o, e = cli.exec_command(
        'sed -n "13,42p" %s/vla_demo/start_demo.sh' % HOME, timeout=30)
    print(o.read().decode('utf-8', 'replace'))
    print('[ERR]', e.read().decode('utf-8', 'replace')[:300])
    cli.close()


if __name__ == '__main__':
    main()
