"""在 VM 侧（Linux, uid 1000）把 P73 行 + §13 写入 TaskPlan.md"""
import io, os, sys, hashlib
import paramiko

HOME = '/home/earl/workspace/VLA'
TARGET = HOME + '/TaskPlan.md'

cli = paramiko.SSHClient()
cli.set_missing_host_key_policy(paramiko.AutoAddPolicy())
cli.connect('192.168.255.128', 22, 'earl', '111111',
            timeout=10, look_for_keys=False, allow_agent=False)
sftp = cli.open_sftp()

with sftp.file(TARGET, 'r') as f:
    s = f.read().decode('utf-8')

before = s
log = []

P73_ROW = (
 '| P73 | 2026-10-25 | 修复+调参 | **距离环人速前馈**（§12.6 建议 1 的落地）。'
 '修掉 3 个真实 bug：①`_gap6` 全局从未赋值 → "真值硬保"分支必现 `NameError`，改 `geo_gap(rx, ry, _obj6)`；'
 '②`path_free` 静默停车 —— `_ok6` 是 1.6m 扫掠布尔量，旁边有柱子即为 False，而 `_obj6 is None` 时'
 '既不写 note 也不给转向，导致 `dh` 恒 0、指令 v 恒 0（日志实证 `dh=0.00 df=1.48` 仍停车），'
 '改为只认真实接触 `_dpath < 0.33`；'
 '③前馈**方向反了** —— 2D 位置差分受 1° 方位量化噪声污染（1.5m 处 ±0.26 m/s），'
 '人正接近（0.83→0.55m）却输出 `ff=+0.50` 全速推，把人顶到 0.41m 占满画幅致整帧漏检，'
 '改为标量观测 `robot_vfwd + d(dist)/dt`。'
 '**诚实结论：指标未改善**，`in_band` 从基线 53.6% 落到 20~39%，居中率从 98.2% 落到 55~93%；'
 '根因是**评测不可重复**（`person_walk` 用 `time.time()` 而非 `/clock`，`PW_SEED=7` 两次跑人轨迹偏差 11m），'
 '无法做 A/B 归因 | 完成（无收益，仅修 bug） | M4/§12.6 |'
)

if '| P73 |' not in s:
    anchor = '全程 pitch=0 | 完成 | M3/P69 |'
    n = s.count(anchor)
    if n != 1:
        print('ANCHOR_FAIL count=%d' % n); sys.exit(1)
    s = s.replace(anchor, anchor + '\n' + P73_ROW, 1)
    log.append('row inserted')
else:
    log.append('row exists, skip')

SEC13 = io.open(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                             'tmp_p73_sec13.md'), encoding='utf-8').read()
if '## 13. 距离环人速前馈' not in s:
    s = s.rstrip('\n') + '\n' + SEC13
    log.append('section 13 appended')
else:
    log.append('section 13 exists, skip')

with sftp.file(TARGET, 'w') as f:
    f.write(s.encode('utf-8'))

with sftp.file(TARGET, 'r') as f:
    back = f.read().decode('utf-8')

print('\n'.join(log))
print('lines %d -> %d' % (before.count('\n') + 1, back.count('\n') + 1))
print('verify P73 row:', back.count('| P73 |'))
print('verify sec13:', back.count('## 13. 距离环人速前馈'))
sftp.close(); cli.close()
