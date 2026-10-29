"""P73e: 去掉 _stight 软闸, 窄道只给 0.40 m/s 上限"""
import paramiko

HOME = '/home/earl/workspace/VLA'
SRC = HOME + '/vla_ws/src/vla_follow/vla_follow/follower.py'
DEP = HOME + '/deploy/follower.py'

cli = paramiko.SSHClient()
cli.set_missing_host_key_policy(paramiko.AutoAddPolicy())
cli.connect('192.168.255.128', 22, 'earl', '111111',
            timeout=10, look_for_keys=False, allow_agent=False)
sftp = cli.open_sftp()
with sftp.file(SRC, 'r') as f:
    s = f.read().decode('utf-8')

EDITS = [
    # A. 去掉 _stight 软闸
    ("            d_head = min(d_f, _dpath)\n"
     "            # P73: 窄段不再假设闸(会死锁), 而是\"在窄段前沿之前 stop_d\n"
     "            # 处开始收速度\" —— 交给下游 v_cap=(d_head-stop_d)/tau 那条\n"
     "            # 斜坡统一处理: 离窄段越近越慢, 贴到前沿才降到 0。\n"
     "            d_head = min(d_head, _stight + stop_d)\n",
     "            # P73: 硬闸只看真实接触距离 _dpath。窄段不能折进 d_head ——\n"
     "            # 机器人**已经**在窄带里时 _stight 从 0.05 起, _stight+stop_d\n"
     "            # 会把 d_head 拉到 0.71, 经 (dh-stop_d)/tau 斜坡后只剩 0.08,\n"
     "            # 等于换了个马甲的死锁(t9 实测 far 63.7%)。窄段只按下面\n"
     "            # 的 0.40m/s 上限限速。\n"
     "            d_head = min(d_f, _dpath)\n"),

    # B. 窄道限速 0.30 → 0.40, 且只在窄段确实在前方 1m 内才限
    ("        if not _okp and v_t > 0.30:\n"
     "            v_t = 0.30\n"
     "            note = (note + ' 窄道限速0.30').strip()\n",
     "        if (not _okp) and _stight < 1.00 and v_t > 0.40:\n"
     "            v_t = 0.40\n"
     "            note = (note + ' 窄道限速0.40').strip()\n"),
]

for i, (old, new) in enumerate(EDITS):
    if s.count(old) != 1:
        raise SystemExit('第 %d 处锚点命中 %d 次, 中止:\n%s'
                         % (i + 1, s.count(old), old[:120]))
    s = s.replace(old, new, 1)

with sftp.file(SRC, 'w') as f:
    f.write(s.encode('utf-8'))
with sftp.file(DEP, 'w') as f:
    f.write(s.encode('utf-8'))

print('已打补丁 %d 处' % len(EDITS))
i, o, e = cli.exec_command(
    'cd %s && python3 -m py_compile %s && echo PY_OK' % (HOME, SRC),
    timeout=60)
print(o.read().decode(), e.read().decode()[:300])
i, o, e = cli.exec_command('md5sum %s %s' % (SRC, DEP), timeout=30)
print(o.read().decode())
sftp.close()
cli.close()
