# -*- coding: utf-8 -*-
"""P72: 底盘运动学标定 —— SDF 物理修复

修三处(gz 日志与实测姿态均已证实):

1. caster 是"没有 joint 的游离 link" → 不支撑车体。
   实测 robot 物理姿态 pitch=+0.540 rad(31°), 车体整个向后仰着在地上拖,
   线速度打滑(1.385m 位移里横向占 1.155m)、角速度符号与幅值全乱。
   补 caster_joint(revolute, 绕 Y 滚动 + mu 0.05 允许转向时侧滑)。

2. 轮 joint 的 <xyz resolved_frame="child"> 里的 resolved_frame 不是 SDF 属性
   (gz 明确告警 "XML Attribute[resolved_frame] not defined in SDF" 后忽略),
   实际轴 = joint 坐标系 0 0 1; 而轮 link 绕 X 转了 +90°, 其 Z 轴指向机体系
   -Y → 正轮速 = 后退。实测: 命令 angular.z=+0.8 得到 -135.6°(反向)。
   改为 0 0 -1 → 机体系 +Y, 正轮速 = 前进, 左右差速 = 逆时针(+yaw)。

3. <axis><limits> 应为 <limit>(gz 同样告警并忽略), 一并改正。

person 模型有完全相同的三处问题(它的轮轴也是 -Y → 人其实是倒着走),
一起修, 并给它补 caster(裙摆底面离地 0.02m, 无支撑同样会后仰)。
"""
import io
import posixpath
import sys

import paramiko

HOST, USER, PWD = '192.168.255.128', 'earl', '111111'
HOME = '/home/earl/workspace/VLA'

SDF = posixpath.join(HOME, 'deploy/work_area.sdf')
BAK = posixpath.join(HOME, '.bak_p0/work_area.sdf.p71')

OLD_AXIS = '<xyz resolved_frame="child">0 0 1</xyz>'
NEW_AXIS = '<xyz>0 0 -1</xyz>'
OLD_LIM = '<limits><lower>-1e16</lower><upper>1e16</upper></limits>'
NEW_LIM = '<limit><lower>-1e16</lower><upper>1e16</upper></limit>'

ROBOT_ANCHOR = """      <joint name="right_wheel_joint" type="revolute">
        <parent>chassis</parent>
        <child>right_wheel</child>
        <axis>
          <xyz>0 0 -1</xyz>
          <limit><lower>-1e16</lower><upper>1e16</upper></limit>
        </axis>
      </joint>
"""

ROBOT_CASTER = ROBOT_ANCHOR + """      <!-- P72: 后支撑脚(原来 caster 是游离 link, 车体 31° 后仰拖地) -->
      <joint name="caster_joint" type="revolute">
        <parent>chassis</parent>
        <child>caster</child>
        <axis>
          <xyz>0 0 -1</xyz>
          <limit><lower>-1e16</lower><upper>1e16</upper></limit>
        </axis>
      </joint>
"""

PERSON_ANCHOR = """      <joint name="right_wheel_joint" type="revolute">
        <parent>skirt</parent>
        <child>right_wheel</child>
        <axis>
          <xyz>0 0 -1</xyz>
          <limit><lower>-1e16</lower><upper>1e16</upper></limit>
        </axis>
      </joint>
"""

PERSON_CASTER = PERSON_ANCHOR + """      <!-- P72: 人的后支撑脚(裙摆底面离地 0.02m, 补支撑防后仰) -->
      <joint name="caster_joint" type="revolute">
        <parent>skirt</parent>
        <child>caster</child>
        <axis>
          <xyz>0 0 -1</xyz>
          <limit><lower>-1e16</lower><upper>1e16</upper></limit>
        </axis>
      </joint>
"""

# person 的 caster link 插在 right_wheel link 之后(与 robot 的 caster 同构)
PERSON_LINK_ANCHOR = """      <link name="right_wheel">
        <pose relative_to="skirt">0 -0.20 -0.225 1.5707 0 0</pose>
        <inertial>
          <mass>0.25</mass>
          <inertia><ixx>0.0003</ixx><iyy>0.0003</iyy><izz>0.0003</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
        </inertial>
        <collision name="c">
          <geometry><cylinder><radius>0.055</radius><length>0.05</length></cylinder></geometry>
          <surface><friction><ode><mu>1.2</mu><mu2>1.0</mu2></ode></friction></surface>
        </collision>
        <visual name="v">
          <geometry><cylinder><radius>0.055</radius><length>0.05</length></cylinder></geometry>
          <material><ambient>0.1 0.1 0.1 1</ambient><diffuse>0.1 0.1 0.1 1</diffuse></material>
        </visual>
      </link>
"""

PERSON_CASTER_LINK = PERSON_LINK_ANCHOR + """      <link name="caster">
        <pose relative_to="skirt">-0.18 0 -0.25 0 0 0</pose>
        <inertial>
          <mass>0.1</mass>
          <inertia><ixx>0.000036</ixx><iyy>0.000036</iyy><izz>0.000036</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
        </inertial>
        <collision name="c">
          <geometry><sphere><radius>0.03</radius></sphere></geometry>
          <surface><friction><ode><mu>0.05</mu><mu2>0.05</mu2></ode></friction></surface>
        </collision>
        <visual name="v">
          <geometry><sphere><radius>0.03</radius></sphere></geometry>
          <material><ambient>0.3 0.3 0.3 1</ambient><diffuse>0.3 0.3 0.3 1</diffuse></material>
        </visual>
      </link>
"""


def patch(text):
    n = 0
    n += text.count(OLD_AXIS)
    text = text.replace(OLD_AXIS, NEW_AXIS)
    m = text.count(OLD_LIM)
    n += m
    text = text.replace(OLD_LIM, NEW_LIM)
    report = ['axis/limit 替换: axis %d 处 + limit %d 处' % (n - m, m)]

    for name, anchor, new in (('robot', ROBOT_ANCHOR, ROBOT_CASTER),
                              ('person', PERSON_ANCHOR, PERSON_CASTER)):
        if 'caster_joint' in text:
            report.append('%s: caster_joint 已存在, 跳过' % name)
            break
        if text.count(anchor) != 1:
            raise SystemExit('锚点不唯一/未找到: %s (count=%d)'
                             % (name, text.count(anchor)))
        text = text.replace(anchor, new, 1)
        report.append('%s: 已补 caster_joint' % name)

    if '<link name="caster">' not in text.split('<model name="person"')[1]:
        if text.count(PERSON_LINK_ANCHOR) != 1:
            raise SystemExit('person caster link 锚点异常')
        text = text.replace(PERSON_LINK_ANCHOR, PERSON_CASTER_LINK, 1)
        report.append('person: 已补 caster link')
    return text, report


def main():
    cli = paramiko.SSHClient()
    cli.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    cli.connect(HOST, 22, USER, PWD, timeout=10,
                look_for_keys=False, allow_agent=False)
    sftp = cli.open_sftp()

    def run(cmd, t=300):
        i, o, e = cli.exec_command(cmd, timeout=t)
        out = o.read().decode('utf-8', 'replace')
        err = e.read().decode('utf-8', 'replace')
        if out.strip():
            print(out.rstrip())
        if err.strip():
            print('[ERR]', err.rstrip()[:800])
        return out

    with sftp.file(SDF, 'r') as f:
        src = f.read().decode('utf-8')

    if 'caster_joint' in src and NEW_AXIS in src:
        print('已打过补丁, 跳过文本修改')
        new = src
    else:
        try:
            run('mkdir -p %s && cp -n %s %s' % (posixpath.dirname(BAK), SDF, BAK))
        except Exception as ex:  # noqa
            print('备份跳过:', ex)
        new, report = patch(src)
        for r in report:
            print(' -', r)
        with sftp.file(SDF, 'w') as f:
            f.write(new.encode('utf-8'))
        print('已写回 deploy/work_area.sdf  (%d → %d bytes)' % (len(src), len(new)))

    # 同步到 src/worlds
    run('cd %s && cp -f deploy/work_area.sdf '
        'vla_ws/src/vla_follow/worlds/work_area.sdf && '
        'md5sum deploy/work_area.sdf vla_ws/src/vla_follow/worlds/work_area.sdf'
        % HOME)

    # XML 合法性 + 关键改动确认
    run("python3 - <<'PYEOF'\n"
        "import xml.etree.ElementTree as ET\n"
        "p='%s'\n"
        "t=ET.parse(p); r=t.getroot()\n"
        "ns='{http://www.w3.org/2006/06/sdformat}'\n" % SDF +
        "for m in r.iter():\n"
        "    if m.tag.endswith('model') and m.get('name') in ('robot','person'):\n"
        "        links=[l.get('name') for l in m if l.tag.endswith('link')]\n"
        "        joints=[(j.get('name'), j.get('type')) for j in m if j.tag.endswith('joint')]\n"
        "        print(m.get('name'), 'links=', links)\n"
        "        print('   joints=', joints)\n"
        "        ax=[a.findtext('{http://www.w3.org/2006/06/sdformat}xyz')\n"
        "            for j in m if j.tag.endswith('joint')\n"
        "            for a in j if a.tag.endswith('axis')]\n"
        "        print('   axes=', ax)\n"
        "print('XML_OK')\n"
        "PYEOF")

    sftp.close()
    cli.close()


if __name__ == '__main__':
    sys.exit(main())
