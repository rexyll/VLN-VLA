# -*- coding: utf-8 -*-
"""P62: 让差速驱动同时满足 ROS 约定(v>0=前进, w>0=左转/逆时针)。

为什么需要这一步:
  P61 把 4 处差速轮 joint axis 由 `0 0 1` 改回 `0 0 -1`, 修好了"前进"
  (实测前向点积 +1.000)。但 gz DiffDrive 的左右轮速度差决定偏航, 两个轮轴
  同时反号 => **前进(共模)与偏航(差模)被一起镜像**, 于是 w>0 变成"右转"。
  而 follower 与 person_walk 都按 ROS 约定写的:
      v>0 = 沿自身朝向前进
      w>0 = 逆时针(左转)   -> w_t = kp * brg, brg>0 表示目标在左侧
  两者必须同时成立, 所以还要单独翻转差模分量。

做法: 交换 DiffDrive 插件里 left_joint / right_joint 的绑定。
  - 前进 = (v_left + v_right)/2 是**共模**分量, 交换左右**不变**
  - 偏航 ∝ (v_right - v_left) 是**差模**分量, 交换左右**变号**
  于是只翻转偏航、不影响前进, 正好补齐 ROS 约定。

物理含义: gz 里 left_joint/right_joint 只是"被驱动的两个关节", 轮子的
左右属性由它们在底盘上的安装位置(y=±0.13 / ±0.20)决定。把名字对调,
等价于"左右轮接反", 差速方向随之反转 —— 纯几何修正, 不涉及物理参数。

robot 与 person 两个模型都要改(person_walk 同样按 ROS 约定)。
"""
import re
import sys

FILES = ['_work_area_p60.sdf', '_work_area_p58.sdf', '_work_area.sdf']

PLUGIN_RE = re.compile(
    r'(<plugin\s+filename="gz-sim-diff-drive-system".*?</plugin>)', re.S)


def fix(path):
    with open(path, 'r', encoding='utf-8') as f:
        src = f.read()

    if '<left_joint>right_wheel_joint</left_joint>' in src:
        print('[skip] %s 已交换过' % path)
        return 0

    n = [0]

    def repl(m):
        blk = m.group(1)
        lj = re.search(r'<left_joint>([^<]*)</left_joint>', blk)
        rj = re.search(r'<right_joint>([^<]*)</right_joint>', blk)
        if not (lj and rj) or lj.group(1) == rj.group(1):
            return blk
        lv, rv = lj.group(1), rj.group(1)
        # 先摘出旧值, 再写回对调后的绑定(用占位符避免二次替换)
        blk2 = blk.replace('<left_joint>%s</left_joint>' % lv,
                           '<left_joint>@L@</left_joint>')
        blk2 = blk2.replace('<right_joint>%s</right_joint>' % rv,
                            '<right_joint>@R@</right_joint>')
        blk2 = blk2.replace('<left_joint>@L@</left_joint>',
                            '<left_joint>%s</left_joint>' % rv)
        blk2 = blk2.replace('<right_joint>@R@</right_joint>',
                            '<right_joint>%s</right_joint>' % lv)
        n[0] += 1
        return blk2

    out = PLUGIN_RE.sub(repl, src)
    if n[0] != 2:
        raise SystemExit('[FAIL] %s 期望交换 2 个 DiffDrive 插件, 实际 %d'
                         % (path, n[0]))
    with open(path, 'w', encoding='utf-8') as f:
        f.write(out)
    print('[ok] %s 交换 %d 个 DiffDrive 的左右轮绑定' % (path, n[0]))
    return n[0]


def verify(path):
    with open(path, 'r', encoding='utf-8') as f:
        src = f.read()
    lj = re.findall(r'<left_joint>([^<]*)</left_joint>', src)
    rj = re.findall(r'<right_joint>([^<]*)</right_joint>', src)
    wheel_neg = len(re.findall(
        r'<joint\s+name="[a-z_]*wheel_joint".*?resolved_frame="child">0 0 -1<',
        src, re.S))
    print('     校验: left_joint=%s right_joint=%s 差速轮轴-1=%d处'
          % (lj, rj, wheel_neg))
    assert len(lj) == 2 and len(rj) == 2, 'DiffDrive 插件数量异常'
    assert lj[0] == 'right_wheel_joint' and rj[0] == 'left_wheel_joint', \
        'robot 左右轮未交换'
    assert lj[1] == 'right_wheel_joint' and rj[1] == 'left_wheel_joint', \
        'person 左右轮未交换'
    assert wheel_neg == 4, '差速轮轴应全部为 -1'
    import xml.etree.ElementTree as ET
    ET.parse(path)
    print('     XML 解析通过')


if __name__ == '__main__':
    total = 0
    for p in (sys.argv[1:] or FILES):
        total += fix(p)
        verify(p)
    print('\n合计交换 %d 个 DiffDrive 插件' % total)
