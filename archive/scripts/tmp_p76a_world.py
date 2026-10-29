"""P76a: 在机器人身上挂"急停圈/限速圈"可视化

用户要在 Gazebo 里实时看到"触发机器人急停的范围"。
直接把两个 visual 挂在 chassis link 上最省事: 跟随零延迟、零服务调用、
不占 dynamic_pose/info(gt_tf 只订阅 dynamic), 也不影响物理(visual 无碰撞).

  stop_v  : 实心红盘   r = lidar_stop_m  0.66  (<= 此距离立即停车)
  slow ring: 36 段虚线圈 r = lidar_slow_m  1.60  (在此之内按距离限速)
  以及 side_v: 细线圈 r = lidar_side_m 0.60 —— 侧向防擦阈值, 与急停圈只差 6cm,
            不单独画, 免得糊在一起。
"""
import math
import sys

import paramiko

HOME = '/home/earl/workspace/VLA'
WORLD = HOME + '/vla_ws/src/vla_follow/worlds/work_area.sdf'

cli = paramiko.SSHClient()
cli.set_missing_host_key_policy(paramiko.AutoAddPolicy())
cli.connect('192.168.255.128', 22, 'earl', '111111',
            timeout=10, look_for_keys=False, allow_agent=False)
sftp = cli.open_sftp()


def rd(p):
    with sftp.file(p, 'r') as f:
        return f.read().decode('utf-8')


def wr(p, s):
    with sftp.file(p, 'w') as f:
        f.write(s.encode('utf-8'))


def rep1(s, old, new, tag):
    n = s.count(old)
    if n != 1:
        print('FAIL[%s] count=%d' % (tag, n))
        sys.exit(1)
    return s.replace(old, new, 1)


STOP_R = 0.66
SLOW_R = 1.60
NSEG = 36

# 急停盘: chassis 在 z=0.13, 盘放 world z≈0.01
stop_block = """        <!-- P76: 急停圈(r=0.66, lidar_stop_m) —— 障碍进入此盘即硬停 -->
        <visual name="stop_zone_v">
          <pose>0 0 -0.12 0 0 0</pose>
          <geometry><cylinder><radius>%.3f</radius><length>0.02</length></cylinder></geometry>
          <material>
            <ambient>1 0.15 0.1 1</ambient>
            <diffuse>1 0.15 0.1 1</diffuse>
            <emissive>0.35 0.02 0.0 1</emissive>
          </material>
          <transparency>0.55</transparency>
          <cast_shadows>false</cast_shadows>
        </visual>
""" % STOP_R

# 限速圈: 36 段小方块组成的虚线圆
ring = []
for i in range(NSEG):
    a = 2.0 * math.pi * i / NSEG
    x = SLOW_R * math.cos(a)
    y = SLOW_R * math.sin(a)
    ring.append(
        '        <visual name="slow_ring_%02d">\n'
        '          <pose>%.4f %.4f -0.105 0 0 %.4f</pose>\n'
        '          <geometry><box><size>0.09 0.035 0.012</size></box></geometry>\n'
        '          <material><ambient>1 0.75 0.1 1</ambient>'
        '<diffuse>1 0.75 0.1 1</diffuse></material>\n'
        '          <transparency>0.30</transparency>\n'
        '          <cast_shadows>false</cast_shadows>\n'
        '        </visual>\n' % (i, x, y, a))
slow_block = ('        <!-- P76: 限速圈(r=1.60, lidar_slow_m) 虚线, 进圈后按距离限速 -->\n'
              + ''.join(ring))

s = rd(WORLD)
if 'stop_zone_v' in s:
    print('SKIP: P76 可视化已存在')
else:
    s = rep1(s, '        <!-- 雷达桅杆(外观) -->\n',
             stop_block + slow_block + '        <!-- 雷达桅杆(外观) -->\n',
             'chassis_visuals')
    wr(WORLD, s)
    print('PATCHED', WORLD)

i, o, e = cli.exec_command(
    'python3 -c "import xml.dom.minidom as m; m.parse(\'%s\'); print(\'XML_OK\')"' % WORLD,
    timeout=60)
print(o.read().decode(errors='replace').strip(),
      e.read().decode(errors='replace')[:300])
i, o, e = cli.exec_command(
    'grep -c "slow_ring_" %s; grep -n "stop_zone_v" %s | head -2' % (WORLD, WORLD),
    timeout=30)
print(o.read().decode(errors='replace').strip())

# 同步到 install 与 demo 副本(如果存在)
i, o, e = cli.exec_command(
    'for p in %s/vla_ws/install/vla_follow/share/vla_follow/worlds/work_area.sdf '
    '%s/demo_follow/vla_ws/src/vla_follow/worlds/work_area.sdf; do '
    '[ -f "$p" ] && cp %s "$p" && echo "synced $p"; done' % (HOME, HOME, WORLD),
    timeout=30)
print(o.read().decode(errors='replace').strip())

sftp.close()
cli.close()
print('DONE')
