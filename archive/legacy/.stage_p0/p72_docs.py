# -*- coding: utf-8 -*-
"""把 P72 条目插入 TaskPlan 进度表, 并把 §12 追加到文末(本地 Z: 只读, 走 VM 侧写)。"""
import io
import posixpath

import paramiko

HOST, USER, PWD = '192.168.255.128', 'earl', '111111'
HOME = '/home/earl/workspace/VLA'
TP = posixpath.join(HOME, 'TaskPlan.md')
STAGE = 'Z:/VLA/.stage_p0/'

row = io.open(STAGE + 'taskplan_p72_row.md', encoding='utf-8').read().strip()
sec = io.open(STAGE + 'taskplan_p72_sec.md', encoding='utf-8').read()

cli = paramiko.SSHClient()
cli.set_missing_host_key_policy(paramiko.AutoAddPolicy())
cli.connect(HOST, 22, USER, PWD, timeout=10,
            look_for_keys=False, allow_agent=False)
sftp = cli.open_sftp()

with sftp.file(TP, 'r') as f:
    txt = f.read().decode('utf-8')
before = len(txt)

if '| P72 |' in txt:
    print('P72 行已存在, 跳过')
else:
    lines = txt.split('\n')
    idx = next((i for i, l in enumerate(lines) if l.startswith('| P70 |')), None)
    if idx is None:
        raise SystemExit('找不到 P70 行')
    lines.insert(idx + 1, row)
    txt = '\n'.join(lines)
    print('P72 行已插到 P70 之后(第 %d 行)' % (idx + 2))

if '## 12. 底盘运动学标定' in txt:
    print('§12 已存在, 跳过')
else:
    txt = txt.rstrip('\n') + '\n' + sec
    print('§12 已追加')

with sftp.file(TP, 'w') as f:
    f.write(txt.encode('utf-8'))
sftp.close()

i, o, e = cli.exec_command(
    'cd %s && wc -c TaskPlan.md; grep -c "| P72 |" TaskPlan.md; '
    'grep -n "^## " TaskPlan.md | tail -4' % HOME, timeout=60)
print(o.read().decode())
print('字节: %d → %d' % (before, len(txt)))
cli.close()
