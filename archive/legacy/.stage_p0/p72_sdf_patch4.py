# -*- coding: utf-8 -*-
"""P72 补丁 v4: 给 robot / person 各补一个**前**支撑脚。

根因: 质心(=车体几何中心, x=0)正好压在轮轴(x=0)上, 支撑点是
"两个轮子 + 一个后支撑脚", 支撑多边形是 x∈[-0.12, 0] —— 质心落在
多边形前沿上, 零稳定裕度。任何扰动(启动加速度/数值噪声)都会让它绕轮轴
前倾, 一直到车头下沿触地(理论 atan(0.08/0.15)=28°, 实测 31°, 因为轮子
略有下沉)。前倾后车头承担主要载荷, 轮子打滑 → 之前测到的
"1.385m 位移里 1.155m 是横滑""角速度增益 1.48×"全是这个引起的。

补前支撑脚后支撑多边形变成 x∈[-0.12, +0.12], 质心居中, 前后都有裕度。
person 同构(裙摆质心也在轮轴正上方), 一起补。
"""
import posixpath
import re

import paramiko

HOST, USER, PWD = '192.168.255.128', 'earl', '111111'
HOME = '/home/earl/workspace/VLA'
SDF = posixpath.join(HOME, 'deploy/work_area.sdf')

FRONT_X = {'chassis': 0.12, 'skirt': 0.18}


def add_front(text, parent):
    """在指定 model 里复制 caster link / joint, 生成 caster_front。"""
    mlink = re.search(r'\n(\s*)<link name="caster">.*?</link>\n', text, re.S)
    if not mlink or 'caster_front' in text:
        return text, False
    block = mlink.group(0)
    indent = mlink.group(1)
    front = block.replace('name="caster"', 'name="caster_front"')
    # 位姿 x 取反(后 → 前)
    front = re.sub(r'(-?[\d.]+)(\s+0\s+-0\.(?:10|25)\s+0 0 0)',
                   lambda m: ('%g' % (FRONT_X[parent],)) + m.group(2),
                   front, count=1)
    text = text.replace(block, block + front, 1)

    mj = re.search(r'\n(\s*)<joint name="caster_joint".*?</joint>\n', text, re.S)
    if not mj:
        return text, False
    jblock = mj.group(0)
    jfront = (jblock.replace('name="caster_joint"', 'name="caster_front_joint"')
              .replace('<child>caster</child>', '<child>caster_front</child>'))
    text = text.replace(jblock, jblock + jfront, 1)
    return text, True


def main():
    cli = paramiko.SSHClient()
    cli.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    cli.connect(HOST, 22, USER, PWD, timeout=10,
                look_for_keys=False, allow_agent=False)
    sftp = cli.open_sftp()

    def run(cmd, t=300, quiet=False):
        i, o, e = cli.exec_command(cmd, timeout=t)
        out = o.read().decode('utf-8', 'replace')
        err = e.read().decode('utf-8', 'replace')
        if not quiet:
            if out.strip():
                print(out.rstrip())
            if err.strip():
                print('[ERR]', err.rstrip()[:600])
        return out

    with sftp.file(SDF, 'r') as f:
        s = f.read().decode('utf-8')

    if 'caster_front' in s:
        print('已存在 caster_front, 跳过')
    else:
        text = s
        for parent in ('chassis', 'skirt'):
            # 只在对应 model 的范围内替换
            start = text.index('<model name="%s"' % ('robot' if parent == 'chassis'
                                                     else 'person'))
            end = text.index('</model>', start)
            seg = text[start:end]
            new_seg, ok = add_front(seg, parent)
            if not ok:
                print('%s: 跳过(已有或未找到 caster 块)' % parent)
            text = text[:start] + new_seg + text[end:]
        with sftp.file(SDF, 'w') as f:
            f.write(text.encode('utf-8'))
        print('已写回 deploy/work_area.sdf')

    run('cd %s && cp -f deploy/work_area.sdf '
        'vla_ws/src/vla_follow/worlds/work_area.sdf' % HOME)
    run("python3 - <<'PYEOF'\n"
        "import xml.etree.ElementTree as ET\n"
        "r = ET.parse('%s').getroot()\n" % SDF +
        "for m in r.iter():\n"
        "    if m.tag.split('}')[-1] != 'model' or m.get('name') not in ('robot','person'):\n"
        "        continue\n"
        "    print(m.get('name'), 'links=', [c.get('name') for c in m\n"
        "          if c.tag.split('}')[-1] == 'link'])\n"
        "    print('   joints=', [c.get('name') for c in m\n"
        "          if c.tag.split('}')[-1] == 'joint'])\n"
        "PYEOF")
    sftp.close()
    cli.close()


if __name__ == '__main__':
    main()
