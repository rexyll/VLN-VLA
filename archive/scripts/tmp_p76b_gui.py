"""P76b: 让 Gazebo 一启动就是全场俯视 + 3D 视图占满窗口

两个问题:
 1) 默认相机贴得太近(实测截图只能看到机器人周围 2m), 看不到轨迹全貌。
    world SDF 里的 <gui><camera><pose> 优先级高于 ~/.gz/sim/8/gui.config,
    写在这里最稳, 且随工程走。
 2) 右侧 EntityTree / ComponentInspector 两块面板占掉一半宽度, 3D 视图只剩
    ~380px, 12m 见方的场地根本放不下。这两个面板对"看轨迹"没用, 去掉。
    原文件备份为 gui.config.bak_p76。
"""
import sys

import paramiko

HOME = '/home/earl/workspace/VLA'
WORLD = HOME + '/vla_ws/src/vla_follow/worlds/work_area.sdf'
GUI = '/home/earl/.gz/sim/8/gui.config'

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


# ---------------------------------------------------------------- 1) world gui
GUIBLK = """
  <!-- P76: 启动即全场俯视。z=13 时 12m 见方场地刚好铺满; 两个圆圈和轨迹点
       都在这个视角下能看清。 -->
  <gui fullscreen="0">
    <camera>
      <pose>0.0 0.0 13.0 0 -1.5707963 0</pose>
      <view_controller>orbit</view_controller>
    </camera>
  </gui>
</sdf>"""

s = rd(WORLD)
if '<gui fullscreen' in s:
    print('SKIP: world gui 已存在')
else:
    if s.count('\n</sdf>') != 1:
        print('FAIL: </sdf> 不唯一')
        sys.exit(1)
    s = s.replace('\n</sdf>', GUIBLK, 1)
    wr(WORLD, s)
    print('PATCHED world gui')

i, o, e = cli.exec_command(
    'python3 -c "import xml.dom.minidom as m; m.parse(\'%s\'); print(\'XML_OK\')"' % WORLD,
    timeout=60)
print(o.read().decode(errors='replace').strip(),
      e.read().decode(errors='replace')[:300])

# ---------------------------------------------------------------- 2) gui.config
g = rd(GUI)
if 'P76' not in g:
    i, o, e = cli.exec_command('cp %s %s.bak_p76 && echo BACKUP' % (GUI, GUI),
                               timeout=30)
    print(o.read().decode(errors='replace').strip())
    import re
    for name in ('ComponentInspector', 'EntityTree'):
        pat = re.compile(r'\n<plugin filename="%s".*?</plugin>\n' % name, re.S)
        n = len(pat.findall(g))
        if n == 1:
            g = pat.sub('\n<!-- P76: %s 面板已移除, 让 3D 视图占满宽度 -->\n'
                        % name, g)
            print('removed plugin', name)
        else:
            print('WARN plugin %s matches=%d' % (name, n))
    g = g.replace('<camera_pose>0.001 -0.001 16 0 -1.56 0</camera_pose>',
                  '<camera_pose>0.0 0.0 13.0 0 -1.5707963 0</camera_pose>')
    wr(GUI, g)
    print('PATCHED gui.config')

# ---------------------------------------------------------------- 3) 轨迹点加大
P = HOME + '/vla_ws/src/vla_follow/vla_follow/gz_viz.py'
v = rd(P)
if 'DOT_R = 0.032' in v:
    v = v.replace('DOT_R = 0.032',
                  'DOT_R = 0.045   # P76b: 俯视 13m 时 0.032 只有 3px, 加大到 5px')
    wr(P, v)
    print('PATCHED DOT_R')

cli.exec_command('for p in %s/vla_ws/install/vla_follow/share/vla_follow/worlds/work_area.sdf '
                 '%s/demo_follow/vla_ws/src/vla_follow/worlds/work_area.sdf; do '
                 '[ -f "$p" ] && cp %s "$p"; done' % (HOME, HOME, WORLD), timeout=30)

sftp.close()
cli.close()
print('DONE')
