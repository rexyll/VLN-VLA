# -*- coding: utf-8 -*-
"""P72 补丁 v2: person 的 caster_joint 漏了(第一版幂等判断写错), 补上并复核轴。"""
import posixpath

import paramiko

HOST, USER, PWD = '192.168.255.128', 'earl', '111111'
HOME = '/home/earl/workspace/VLA'
SDF = posixpath.join(HOME, 'deploy/work_area.sdf')

ANCHOR = """      <joint name="right_wheel_joint" type="revolute">
        <parent>skirt</parent>
        <child>right_wheel</child>
        <axis>
          <xyz>0 0 -1</xyz>
          <limit><lower>-1e16</lower><upper>1e16</upper></limit>
        </axis>
      </joint>
"""

CASTER = ANCHOR + """      <joint name="caster_joint" type="revolute">
        <parent>skirt</parent>
        <child>caster</child>
        <axis>
          <xyz>0 0 -1</xyz>
          <limit><lower>-1e16</lower><upper>1e16</upper></limit>
        </axis>
      </joint>
"""

VERIFY = '''python3 - <<'PYEOF'
import xml.etree.ElementTree as ET
r = ET.parse('%(p)s').getroot()
def txt(e, name):
    for c in e:
        if c.tag.split('}')[-1] == name:
            return c
    return None
for m in r.iter():
    if m.tag.split('}')[-1] != 'model' or m.get('name') not in ('robot', 'person'):
        continue
    print(m.get('name'), 'links=', [c.get('name') for c in m
                                    if c.tag.split('}')[-1] == 'link'])
    for j in m:
        if j.tag.split('}')[-1] != 'joint':
            continue
        ax = txt(j, 'axis')
        xyz = txt(ax, 'xyz') if ax is not None else None
        lim = txt(ax, 'limit') if ax is not None else None
        print('   %-17s %-9s axis=%-8s limit=%s'
              % (j.get('name'), j.get('type'),
                 xyz.text if xyz is not None else 'NONE',
                 'ok' if lim is not None else 'MISSING'))
PYEOF'''

cli = paramiko.SSHClient()
cli.set_missing_host_key_policy(paramiko.AutoAddPolicy())
cli.connect(HOST, 22, USER, PWD, timeout=10,
            look_for_keys=False, allow_agent=False)
sftp = cli.open_sftp()


def run(cmd, t=120):
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

part = s.split('<model name="person"')[1]
if 'caster_joint' in part:
    print('person caster_joint 已存在')
elif s.count(ANCHOR) == 1:
    s = s.replace(ANCHOR, CASTER, 1)
    with sftp.file(SDF, 'w') as f:
        f.write(s.encode('utf-8'))
    print('person: caster_joint 已补')
else:
    print('锚点异常 count=%d' % s.count(ANCHOR))
    raise SystemExit(1)

run('cd %s && cp -f deploy/work_area.sdf vla_ws/src/vla_follow/worlds/work_area.sdf '
    '&& md5sum deploy/work_area.sdf vla_ws/src/vla_follow/worlds/work_area.sdf' % HOME)
run(VERIFY % {'p': SDF})
sftp.close()
cli.close()
