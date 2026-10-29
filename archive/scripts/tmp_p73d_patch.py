"""P73d: 窄道只限速不设闸(消除死锁), 并收紧护障裕量到 0.20m"""
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

EDITS = []

# A. 护障裕量 0.10 → 0.20 (车心净空 0.38m = 表面 0.20m)
EDITS.append((
    "def path_free(x, y, ya, margin=0.10, s_max=1.60, step=0.05):\n",
    "def path_free(x, y, ya, margin=0.20, s_max=1.60, step=0.05):\n",
))

# B. section 6: 只认真接触, 窄段交给安全层限速
EDITS.append((
    "                if _dpath < 0.33 or _s6 < 0.30:\n"
    "                    v_t = 0.0\n"
    "                    if isinstance(_obj6, str):\n"
    "                        note = (note + ' 墙前硬停').strip()\n"
    "                    elif _obj6 is None:\n"
    "                        note = (note + ' 窄道硬停 前沿%.2f' % _s6).strip()\n"
    "                    elif _obj6 is not None:\n",

    "                # P73 第二次修正: 不要用 \"窄段前沿 <0.30m\" 做硬停 ——\n"
    "                # _s6 是\"沿航向第一次净空不足\"的弧长, 机器人**已经在**\n"
    "                # 窄带里时(侧方就是立柱) _s6 从 0.05 起步, 这条判据等于\n"
    "                # 永久冻结。实测 t8: 窄道硬停 50 次, 人从 1.0m 走到 5.7m\n"
    "                # 而机器人一动不动。硬停只保留\"马上要接触\"(_dpath<0.33),\n"
    "                # 窄段本身交安全层限速(见 _safety_gate)。\n"
    "                if _dpath < 0.33:\n"
    "                    v_t = 0.0\n"
    "                    if isinstance(_obj6, str):\n"
    "                        note = (note + ' 墙前硬停').strip()\n"
    "                    elif _obj6 is not None:\n",
))

# C. 安全层: 窄段折进 d_head 做分级, 并单独给窄道限速上限
EDITS.append((
    "            d_head = min(d_f, _dpath)\n"
    "            if _stight < 0.30:\n"
    "                d_head = min(d_head, _stight)\n",

    "            d_head = min(d_f, _dpath)\n"
    "            # P73: 窄段不再假设闸(会死锁), 而是\"在窄段前沿之前 stop_d\n"
    "            # 处开始收速度\" —— 交给下游 v_cap=(d_head-stop_d)/tau 那条\n"
    "            # 斜坡统一处理: 离窄段越近越慢, 贴到前沿才降到 0。\n"
    "            d_head = min(d_head, _stight + stop_d)\n",
))

# D. 窄道限速上限(在任何分支前生效)
EDITS.append((
    "        self._dbg.update(dh=d_head, df=d_f)\n"
    "        if v_t > 0.02:                                  # ---- 前进 ----\n",

    "        self._dbg.update(dh=d_head, df=d_f)\n"
    "        # P73: 沿途存在\"车心净空 < ROBOT_R+0.20\"的窄段时只限速不限行。\n"
    "        # 0.30m/s 下 10Hz 深度 + 0.04m 制动距离, 留得下反应余量;\n"
    "        # 而原来的硬停会让\"旁边有柱子\"变成永久冻结(实测 50 次死锁)。\n"
    "        if not _okp and v_t > 0.30:\n"
    "            v_t = 0.30\n"
    "            note = (note + ' 窄道限速0.30').strip()\n"
    "        if v_t > 0.02:                                  # ---- 前进 ----\n",
))

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
print(o.read().decode(), e.read().decode()[:400])
i, o, e = cli.exec_command('md5sum %s %s' % (SRC, DEP), timeout=30)
print(o.read().decode())
sftp.close()
cli.close()
