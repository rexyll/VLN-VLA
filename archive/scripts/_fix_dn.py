import ast
import io

p = '_p43_follower.py'
s = io.open(p, encoding='utf-8').read()
old = "            note = ''\n        if note:\n            err_yaw"
new = ("            note = ''\n"
       "        if _dn:            # P43: 切线绕障标记盖过航点的空标记\n"
       "            note = _dn\n"
       "        if note:\n            err_yaw")
n = s.count(old)
print('match', n)
assert n == 1
s = s.replace(old, new)
io.open(p, 'w', encoding='utf-8', newline='\n').write(s)
ast.parse(s)
print('OK _dn injected')
