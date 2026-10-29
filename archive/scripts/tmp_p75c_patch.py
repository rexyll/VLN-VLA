"""P75c: 修 P75b 的时间基错位

P75b 把录制文件的 t 改成绝对仿真时间(9.015 起), 但 _replay_tick 里仍然拿
相对 tau 与 seq[0][0] 比较 => 要等到 tau=9.015 才开始下发, 人的前 9 秒
(包含 3s WAIT + 起步) 被整段跳过。

改法: 在 _load_replay 里按 hdr.t0_abs 把 t 重基到 0, 外部逻辑不用动。
"""
import sys

import paramiko

HOME = '/home/earl/workspace/VLA'
SRC = HOME + '/vla_ws/src/vla_follow/vla_follow/'
DEP = HOME + '/deploy/'

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


def rep1(s, old, new, tag):
    n = s.count(old)
    if n != 1:
        print('FAIL[%s] count=%d' % (tag, n))
        sys.exit(1)
    return s.replace(old, new, 1)


P = SRC + 'person_walk.py'
s = rd(P)

if 'P75c' in s:
    print('SKIP: P75c 已存在')
else:
    s = rep1(s,
             """        seq.sort(key=lambda r: r[0])
        return seq, hdr
""",
             """        seq.sort(key=lambda r: r[0])
        # P75c: 把绝对时间基重基到 0 —— 调用方一律用相对 tau,
        # 否则 seq[0][0] 是绝对时刻(约 9s), 回放要等 9s 才开始。
        if hdr and 't0_abs' in hdr:
            try:
                _b = float(hdr['t0_abs'])
                seq = [(t - _b, v, w) for (t, v, w) in seq]
            except (TypeError, ValueError):
                pass
        return seq, hdr
""",
             'rebase')
    wr(P, s)
    print('PATCHED', P)
    i, o, e = cli.exec_command(
        'cd %s && python3 -m py_compile person_walk.py && echo SYNTAX_OK' % SRC,
        timeout=60)
    print(o.read().decode(errors='replace').strip(),
          e.read().decode(errors='replace')[:400])
    i, o, e = cli.exec_command('cp %s %sperson_walk.py && echo SYNCED' % (P, DEP),
                               timeout=30)
    print(o.read().decode(errors='replace').strip())

sftp.close()
cli.close()
print('DONE')
