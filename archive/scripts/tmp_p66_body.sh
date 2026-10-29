#!/bin/bash
# tmp_p66_body.sh —— T6 修复: person 模型 torso/head 缺少 joint 导致掉落
set -e
SDF=/home/earl/workspace/VLA/vla_ws/src/vla_follow/worlds/work_area.sdf
python3 - <<'PYEOF'
p = '/home/earl/workspace/VLA/vla_ws/src/vla_follow/worlds/work_area.sdf'
s = open(p, encoding='utf-8').read()
if 'torso_fixed_joint' in s:
    print('已存在, 跳过'); raise SystemExit
anchor = '''      <joint name="left_wheel_joint" type="revolute">
        <parent>skirt</parent>'''
fix = '''      <!-- T6 修复: 躯干/头部此前无 joint 连接, 在 gz-sim 中是自由体直接掉落 -->
      <joint name="torso_fixed_joint" type="fixed">
        <parent>skirt</parent>
        <child>torso</child>
      </joint>
      <joint name="head_fixed_joint" type="fixed">
        <parent>torso</parent>
        <child>head</child>
      </joint>
      <joint name="left_wheel_joint" type="revolute">
        <parent>skirt</parent>'''
assert anchor in s, 'person wheel joint anchor not found'
s = s.replace(anchor, fix, 1)
open(p, 'w', encoding='utf-8').write(s)
print('person torso/head fixed joints 已加入')
PYEOF
cp -f "$SDF" /home/earl/workspace/VLA/deploy/work_area.sdf
echo P66-BODY-DONE
