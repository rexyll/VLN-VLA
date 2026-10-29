"""P73a: 给 follower 的控制链加诊断 + 修掉 _gap6 未定义"""
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

# 1) __init__ 增加诊断字典
EDITS.append((
    "        self._last_note = ''          # P53: 最近一次避障决策说明\n",
    "        self._last_note = ''          # P53: 最近一次避障决策说明\n"
    "        # P73: 距离环控制链快照(仅用于日志诊断)。v_t 最终为 0 时,\n"
    "        # 看 ff/pid/cap/dh 谁先归零, 就能定位是哪一层把它掐掉的。\n"
    "        self._dbg = {'ff': 0.0, 'pid': 0.0, 'cap': 0.0,\n"
    "                     'dh': -1.0, 'df': -1.0}\n",
))

# 2) TRACK 距离环出口记录三个分量
EDITS.append((
    "        _vback = -0.55 if err_d < -0.12 else -0.35\n"
    "        v_t = max(_vback, min(v_cap, v_pid))\n",
    "        _vback = -0.55 if err_d < -0.12 else -0.35\n"
    "        v_t = max(_vback, min(v_cap, v_pid))\n"
    "        self._dbg.update(ff=v_ff, pid=v_pid, cap=v_cap,\n"
    "                         err=err_d)\n",
))

# 3) 安全层记录 d_head / d_f
EDITS.append((
    "        if v_t > 0.02:                                  # ---- 前进 ----\n",
    "        self._dbg.update(dh=d_head, df=d_f)\n"
    "        if v_t > 0.02:                                  # ---- 前进 ----\n",
))

# 4) 日志追加控制链
EDITS.append((
    "                f'w={self._last_w:+.1f} {note}')\n",
    "                f'w={self._last_w:+.1f} {note}'\n"
    "                f' |ff={self._dbg[\"ff\"]:+.2f}'\n"
    "                f' pid={self._dbg[\"pid\"]:+.2f}'\n"
    "                f' cap={self._dbg[\"cap\"]:+.2f}'\n"
    "                f' dh={self._dbg[\"dh\"]:.2f}'\n"
    "                f' df={self._dbg[\"df\"]:.2f}'\n"
    "                f' err={self._dbg.get(\"err\", 0.0):+.2f}')\n",
))

# 5) 修 _gap6 未定义(必现 NameError)
EDITS.append((
    "                        note = (note + ' 真值硬保(%+.1f,%+.1f)间隙%.2f%s'\n"
    "                                % (_obj6[1], _obj6[2], _gap6,\n",
    "                        note = (note + ' 真值硬保(%+.1f,%+.1f)间隙%.2f%s'\n"
    "                                % (_obj6[1], _obj6[2],\n"
    "                                   geo_gap(rx, ry, _obj6),\n",
))

for old, new in EDITS:
    if s.count(old) != 1:
        raise SystemExit('锚点命中 %d 次, 中止:\n%s' % (s.count(old), old[:90]))
    s = s.replace(old, new, 1)

with sftp.file(SRC, 'w') as f:
    f.write(s.encode('utf-8'))
with sftp.file(DEP, 'w') as f:
    f.write(s.encode('utf-8'))
sftp.close()

print('已打补丁 %d 处' % len(EDITS))
i, o, e = cli.exec_command(
    'cd %s && python3 -m py_compile %s && echo PY_OK' % (HOME, SRC),
    timeout=60)
print(o.read().decode(), e.read().decode()[:400])

i, o, e = cli.exec_command('md5sum %s %s' % (SRC, DEP), timeout=30)
print(o.read().decode())
cli.close()
