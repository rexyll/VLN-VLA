# -*- coding: utf-8 -*-
"""把 §11 追加到 TaskPlan.md（本地 Samba 对已有文件只读，故在 VM 侧写）"""
import io
import os

import paramiko

HOST, USER, PWD = '192.168.255.128', 'earl', '111111'
HOME = '/home/earl/workspace/VLA'


def main():
    cli = paramiko.SSHClient()
    cli.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    cli.connect(HOST, 22, USER, PWD, timeout=10,
                look_for_keys=False, allow_agent=False)
    sftp = cli.open_sftp()

    base = os.path.dirname(os.path.abspath(__file__))
    add = io.open(os.path.join(base, '.stage_p0', 'taskplan_append.md'),
                  encoding='utf-8').read()

    p = HOME + '/TaskPlan.md'
    with sftp.file(p, 'r') as f:
        txt = f.read().decode('utf-8')

    if '## 11. P0 执行记录与验收' in txt:
        print('TaskPlan 已含 §11, 改为替换旧版本')
        i0 = txt.index('## 11. P0 执行记录与验收')
        txt = txt[:i0]
    txt = txt.rstrip('\n') + '\n' + add
    with sftp.file(p, 'w') as f:
        f.write(txt.encode('utf-8'))
    print('TaskPlan.md 已写入, 字符数 %d' % len(txt))

    # ---- memory ----
    mem_add = io.open(os.path.join(base, '.stage_p0', 'memory_add.md'),
                      encoding='utf-8').read()
    mp = HOME + '/.workbuddy/memory/2026-10-25.md'
    with sftp.file(mp, 'r') as f:
        m = f.read().decode('utf-8')
    if '### P71/P72 P0 三项落地' in m:
        print('memory 已含本段, 跳过')
    else:
        with sftp.file(mp, 'w') as f:
            f.write((m.rstrip('\n') + '\n' + mem_add).encode('utf-8'))
        print('memory 已追加, 字符数 %d' % (len(m) + len(mem_add)))
    sftp.close()

    _i, o, e = cli.exec_command(
        'cd %s && grep -n "^## 11" TaskPlan.md; wc -l TaskPlan.md; '
        'grep -c "P71/P72" .workbuddy/memory/2026-10-25.md' % HOME,
        timeout=30)
    print(o.read().decode('utf-8', 'replace'))
    print('[ERR]', e.read().decode('utf-8', 'replace')[:300])
    cli.close()


if __name__ == '__main__':
    main()
