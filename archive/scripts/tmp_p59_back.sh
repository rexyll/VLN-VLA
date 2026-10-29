#!/bin/bash
# tmp_p59_back.sh —— P59: person_walk 增加 BACK(后退)行为 + deploy/follower 同步到 src
set -e
W=/home/earl/workspace/VLA
cp -f "$W/deploy/follower.py" "$W/vla_ws/src/vla_follow/vla_follow/follower.py"
echo "follower v4 已同步到 src"

python3 - <<'PYEOF'
import re
p = '/home/earl/workspace/VLA/vla_ws/src/vla_follow/vla_follow/person_walk.py'
s = open(p, encoding='utf-8').read()
if "'BACK'" in s:
    print('BACK 已存在, 跳过')
else:
    # 1) _next_state 的 WALK 分支: 在 TURN(55%) 与 SLOW(78%) 之间切出 BACK(55~70%)
    old = """            if r < 0.55:
                rate = random.uniform(0.3, 0.7) * random.choice([-1, 1])
                angle = random.uniform(0.35, 1.6)
                self.state = 'TURN'
                self.turn_rate = rate
                self.target_speed = 0.15
                self.state_until = time.monotonic() + abs(angle / rate)
                self.get_logger().info(f'PERSON 转向 {math.degrees(angle):.0f}\u00b0')"""
    new = """            if r < 0.45:
                rate = random.uniform(0.3, 0.7) * random.choice([-1, 1])
                angle = random.uniform(0.35, 1.6)
                self.state = 'TURN'
                self.turn_rate = rate
                self.target_speed = 0.15
                self.state_until = time.monotonic() + abs(angle / rate)
                self.get_logger().info(f'PERSON 转向 {math.degrees(angle):.0f}\u00b0')
            elif r < 0.60:
                # P59: 后退行为 —— 人倒退 0.8~2.0s(负线速度), 倒车前先确认
                # 身后 1.5s 预判无危险(倒车方向=当前朝向的反向)
                bx = x - math.cos(self.yaw) * 0.15 * 1.5
                by = y - math.sin(self.yaw) * 0.15 * 1.5
                if in_danger(bx, by, 0.25):
                    self.state_until = time.monotonic() + 1.0
                    return
                self.state = 'BACK'
                self.turn_rate = 0.0
                self.target_speed = -0.18
                self.state_until = time.monotonic() + random.uniform(0.8, 2.0)
                self.get_logger().info('PERSON 后退')"""
    assert old in s, 'WALK branch pattern not found'
    s = s.replace(old, new)
    # 2) 卡死检测状态集合加入 BACK
    old2 = "if self.state in ('WALK', 'TURN', 'SLOW'):"
    assert old2 in s
    s = s.replace(old2, "if self.state in ('WALK', 'TURN', 'SLOW', 'BACK'):")
    # 3) _pick_walk 从 BACK 退出时 target_speed 需为正(已是 random.uniform 正值, 无需改)
    open(p, 'w', encoding='utf-8').write(s)
    print('person_walk BACK 行为已加入')
PYEOF
cp -f "$W/vla_ws/src/vla_follow/vla_follow/person_walk.py" "$W/deploy/person_walk.py"
python3 -c "import ast; ast.parse(open('$W/vla_ws/src/vla_follow/vla_follow/person_walk.py', encoding='utf-8').read()); print('SYNTAX OK')"
source /opt/ros/jazzy/setup.bash
cd "$W/vla_ws" && colcon build 2>&1 | tail -1
echo P59-DONE
