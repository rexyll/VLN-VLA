# -*- coding: utf-8 -*-
"""P72 补丁 v3: caster_joint 的轴改为 0 1 0。

caster link 没有 <pose> 旋转(不像轮子绕 X 转了 90°), 它的 Z 轴就是竖直轴;
轴写成 0 0 -1 会变成"绕竖轴打转"的假万向轮。球体只需能前后滚动,
侧向靠 mu=0.05 滑动 → 轴取机体系 +Y, 即 caster 局部系 0 1 0。
"""
import posixpath

import paramiko

HOST, USER, PWD = '192.168.255.128', 'earl', '111111'
HOME = '/home/earl/workspace/VLA'
SDF = posixpath.join(HOME, 'deploy/work_area.sdf')

OLD = """        <child>caster</child>
        <axis>
          <xyz>0 0 -1</xyz>"""
NEW = """        <child>caster</child>
        <axis>
          <xyz>0 1 0</xyz>"""

cli = paramiko.SSHClient()
cli.set_missing_host_key_policy(paramiko.AutoAddPolicy())
cli.connect(HOST, 22, USER, PWD, timeout=10,
            look_for_keys=False, allow_agent=False)
sftp = cli.open_sftp()


def run(cmd, t=400):
    i, o, e = cli.exec_command(cmd, timeout=t)
    out = o.read().decode('utf-8', 'replace')
    err = e.read().decode('utf-8', 'replace')
    if out.strip():
        print(out.rstrip())
    if err.strip():
        print('[ERR]', err.rstrip()[:600])
    return out


with sftp.file(SDF, 'r') as f:
    s = f.read().decode('utf-8')
print('命中 caster 轴 %d 处' % s.count(OLD))
if s.count(OLD) == 2:
    s = s.replace(OLD, NEW)
    with sftp.file(SDF, 'w') as f:
        f.write(s.encode('utf-8'))
    print('已改为 0 1 0')

run('cd %s && cp -f deploy/work_area.sdf vla_ws/src/vla_follow/worlds/work_area.sdf'
    % HOME)
run("awk '/caster_joint/,/<\\/joint>/' %s/deploy/work_area.sdf" % HOME)
run('md5sum %s/deploy/work_area.sdf %s/vla_ws/src/vla_follow/worlds/work_area.sdf'
    % (HOME, HOME))
sftp.close()
cli.close()
