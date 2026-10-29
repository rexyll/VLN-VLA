import ast
import io

p = '_p43_follower.py'
s = io.open(p, encoding='utf-8').read()

# 1) 切线绕障默认关闭（eval11 实测会原地打转）
old = "        if _blocked:"
new = ("        # P43b: 切线绕障实测会原地打转 —— 侧向余量不足时 v_t 被置 0,\n"
       "        # 只剩 w=±1.2 旋转（日志: 绕障左 前1.20 指令v=+0.00 w=+1.1,\n"
       "        # 人被拉到 4.3m）。启发式局部规划不可靠, 默认关闭,\n"
       "        # 绕障仍需一个真正的局部规划器。保留开关便于后续验证。\n"
       "        if self.cfg['use_tangent'] and _blocked:")
assert s.count(old) == 1, s.count(old)
s = s.replace(old, new, 1)

# 2) 配置项
old2 = "    'use_waypoint': False,          # P39: 航点绕障在临界角上会左右翻转死锁"
new2 = ("    'use_waypoint': False,          # P39: 航点绕障在临界角上会左右翻转死锁\n"
        "    'use_tangent': False,           # P43b: 切线绕障(实测会原地打转, 默认关)")
assert s.count(old2) == 1, s.count(old2)
s = s.replace(old2, new2, 1)

io.open(p, 'w', encoding='utf-8', newline='\n').write(s)
ast.parse(s)
print('OK: 切线绕障已默认关闭, 语法通过')
