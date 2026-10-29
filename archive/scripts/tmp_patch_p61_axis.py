# -*- coding: utf-8 -*-
"""P61: 修复 SDF 差速轮 joint 轴反号。

背景:
  gz DiffDrive 通过 left_joint/right_joint 的 SetVelocity 驱动车轮, 关节轴的
  符号决定了 "cmd_vel.x>0" 对应前进还是后退。P58 起重写 SDF 时把 robot 与
  person 的 4 处差速轮轴从 `0 0 -1` 改成了 `0 0 1`, 导致:
    - 机器人 "前进" 实际后退  (实测 Δpos 与车头差 180°)
    - 人     "前进" 实际后退  (实测 1000+ 帧里移动方向·自身朝向 = -0.937)
    - 转向符号随之翻转      (实测 dyaw/dt~bearing 回归斜率 -0.040, 应为 +1.45)

修复: 只把 4 处 left/right_wheel_joint 的 axis 改回 `0 0 -1`;
      caster_joint / caster_front_joint 是自由万向轮(无驱动扭矩), 符号无功能
      影响, 保持原样不动。
"""
import re
import sys

TARGETS = {
    '_work_area_p60.sdf': 4,
    '_work_area_p58.sdf': 4,
    '_work_area.sdf': 4,
}

JOINT_RE = re.compile(
    r'(<joint\s+name="(?P<jname>[a-z_]*wheel_joint)"[^>]*>.*?</joint>)',
    re.S)


def fix_file(path, expect):
    with open(path, 'r', encoding='utf-8') as f:
        src = f.read()
    if 'resolved_frame="child">0 0 -1<' in src:
        print('[skip] %s 已经是 -1, 不再处理' % path)
        return 0

    n = [0]

    def repl(m):
        blk = m.group(1)
        if 'resolved_frame="child">0 0 1<' not in blk:
            return blk
        blk2 = blk.replace('resolved_frame="child">0 0 1<',
                           'resolved_frame="child">0 0 -1<')
        n[0] += 1
        return blk2

    out = JOINT_RE.sub(repl, src)
    if n[0] != expect:
        raise SystemExit('[FAIL] %s 期望改 %d 处, 实际改 %d 处'
                         % (path, expect, n[0]))
    with open(path, 'w', encoding='utf-8') as f:
        f.write(out)
    print('[ok] %s 修正 %d 处差速轮轴 (0 0 1 -> 0 0 -1)' % (path, n[0]))
    return n[0]


def verify(path):
    with open(path, 'r', encoding='utf-8') as f:
        src = f.read()
    wheel_neg = len(re.findall(
        r'<joint\s+name="[a-z_]*wheel_joint".*?resolved_frame="child">0 0 -1<',
        src, re.S))
    wheel_pos = len(re.findall(
        r'<joint\s+name="[a-z_]*wheel_joint".*?resolved_frame="child">0 0 1<',
        src, re.S))
    caster = re.findall(
        r'<joint\s+name="caster[a-z_]*_joint".*?resolved_frame="child">([^<]*)<',
        src, re.S)
    print('     校验: 差速轮轴 -1=%d 处, +1=%d 处, caster 轴=%s'
          % (wheel_neg, wheel_pos, caster))
    assert wheel_neg == 4 and wheel_pos == 0, '差速轮轴未全部修正'
    # XML 结构完整性
    import xml.etree.ElementTree as ET
    ET.parse(path)
    print('     XML 解析通过')


if __name__ == '__main__':
    total = 0
    for p, e in TARGETS.items():
        total += fix_file(p, e)
        verify(p)
    print('\n合计修正 %d 处' % total)
