"""P73f: 加"接近率闸" —— 不许比补到 0.90m 所需更快地逼近"""
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

OLD = (
    "        # P73: 近距离硬下界。人体近距离会把画面占满(W_REAL 的 55% 容差\n"
    "        # 直接失效), 一旦挤进 0.8m 内就必然整帧判未检出 → 丢失 → 扫描。\n"
    "        # 前馈/距离环任何一处算错都不许越过这条线; 真要更近就由 8 段\n"
    "        # 防撞人(dist<guard_d)去发后退。\n"
    "        if dist < 0.82:\n"
    "            v_cap = 0.0\n"
)
NEW = (
    "        # P73: 接近率闸。前馈用 0.7s 窗口估人速, 人随机行走一转身就有\n"
    "        # ~0.35s 的方向滞后; 这期间指令仍按\"人还在远离\"给速度, 于是每\n"
    "        # 次折返都往人身上多推 0.1~0.15m —— 实测 near(<0.9m) 占 23.7%,\n"
    "        # 最贴近 0.41m。这里按\"在 0.45s 内补到 0.90m 就够\"限制接近率:\n"
    "        #   v - v_person <= (dist-0.90)/0.45\n"
    "        # 只在 dist<1.05 时起作用, 不影响追赶。\n"
    "        room2 = max(0.02, dist - 0.90)\n"
    "        v_cap = min(v_cap, max(0.0, v_ff + room2 / 0.45))\n"
    "        # P73: 近距离硬下界。人体近距离会把画面占满(W_REAL 的 55% 容差\n"
    "        # 直接失效), 一旦挤进 0.8m 内就必然整帧判未检出 → 丢失 → 扫描。\n"
    "        # 前馈/距离环任何一处算错都不许越过这条线; 真要更近就由 8 段\n"
    "        # 防撞人(dist<guard_d)去发后退。\n"
    "        if dist < 0.82:\n"
    "            v_cap = 0.0\n"
)

if s.count(OLD) != 1:
    raise SystemExit('锚点命中 %d 次' % s.count(OLD))
s = s.replace(OLD, NEW, 1)

with sftp.file(SRC, 'w') as f:
    f.write(s.encode('utf-8'))
with sftp.file(DEP, 'w') as f:
    f.write(s.encode('utf-8'))

print('已打补丁')
i, o, e = cli.exec_command(
    'cd %s && python3 -m py_compile %s && echo PY_OK' % (HOME, SRC), timeout=60)
print(o.read().decode(), e.read().decode()[:300])
sftp.close()
cli.close()
