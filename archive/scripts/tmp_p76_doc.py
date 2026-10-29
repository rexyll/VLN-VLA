"""P75/P76 写入 TaskPlan.md（行 + §15 + §16）+ 今日 memory 追加

TaskPlan.md 与 .workbuddy/memory/*.md 的 ACL 只给 Linux 侧 uid 1000 写权限,
Windows 侧只能读, 所以必须从虚拟机侧写。
"""
import io
import os
import sys

import paramiko

HOME = '/home/earl/workspace/VLA'
TP = HOME + '/TaskPlan.md'
MEM = HOME + '/.workbuddy/memory/2026-10-26.md'

cli = paramiko.SSHClient()
cli.set_missing_host_key_policy(paramiko.AutoAddPolicy())
cli.connect('192.168.255.128', 22, 'earl', '111111',
            timeout=10, look_for_keys=False, allow_agent=False)
sftp = cli.open_sftp()

_HERE = os.path.dirname(os.path.abspath(__file__))
for f in ('tmp_p75_sec15.md', 'tmp_p76_sec16.md'):
    sftp.put(os.path.join(_HERE, f), '/tmp/' + f)


def rd(p):
    with sftp.file(p, 'r') as f:
        return f.read().decode('utf-8')


def wr(p, s):
    with sftp.file(p, 'w') as f:
        f.write(s.encode('utf-8'))


sec15 = open(os.path.join(_HERE, 'tmp_p75_sec15.md'), encoding='utf-8').read()
sec16 = open(os.path.join(_HERE, 'tmp_p76_sec16.md'), encoding='utf-8').read()

ROW75 = (
    '| P75 | 2026-10-26 | 方法学 | **评测可重复性 v2：开环回放**。P74 把时间源切到 '
    '`/clock` 后仍缓慢分叉，根因是**人本身闭环**（`robot_close<0.70m` 触发 ESCAPE、'
    '前方 1.5s 预测点离机器人 <1.3m 就转向）—— 改机器人代码必然改变人的路径，种子固定'
    '也没用。新增 `PW_RECORD` / `PW_REPLAY`：录制每拍实际下发的 `(t,v,w)`，回放按时间轴'
    '开环重发，状态机整段跳过。三个坑：①`tau` 锚在节点启动时刻 = 锚在抖动上（两次回放'
    '差 2.6s）→ 改锚**绝对仿真时间**；②`t` 改绝对后没重基到 0，回放要等 9s 才开始、'
    '前段整段被跳过；③手写场景文件 `t0_abs` 只能写 0，需按节点首拍兜底。结果：**回放 vs '
    '回放 mean 0.0011m / max 0.0136m，全程从未超 0.05m，判定 PASS**（改前 0.185/0.573）；'
    '两次 120s 评测采样数完全相同（n=2442）。录制 vs 回放仍有 0.24m（物理接触耦合），'
    '故**回放口径内可做 A/B，绝对数值不可与随机行走口径混用** | 完成 | §15 |\n'
)

ROW76 = (
    '| P76 | 2026-10-26 | 可视化+诊断 | **Gazebo 可视化 + 障碍旁徘徊根因**。'
    '①可视化：本机 gz-sim 8 **没有 marker-manager 插件**，`Marker` 传不进 Gazebo，'
    '改用「chassis 上的 SDF visual + 静态小球 set_pose」——急停圈 r=0.66 半透明红盘、'
    '限速圈 r=1.60 黄色虚线圆（36 段方块）、`gz_viz.py` 用 110 个 static 小球画实际轨迹'
    '（蓝=行进/红=停住）与 Nav2 规划路径（品红）；静态模型不进 `dynamic_pose/info`，'
    '不污染 `gt_tf` 真值链路。②诊断：新增 `/follower/diag` 10Hz JSON，导出安全层内部量；'
    '用 `PW_REPLAY` 手写受控轨迹（人绕柱 ob3 后站定 45s）复现问题。③**根因三条**：'
    '(a)`后退受阻` 一刀切 `v=0` 且无任何兜底 → 人 0.67m 在前、柱 0.44m 在后时**冻结 54.9s**；'
    '(b)`扫描` 写死 `w=+1.20` 与 `对正` 按 brg 符号选向**反向对打**，目标在正后方时 w 在'
    '+1.20/−1.50 之间每 1~3s 翻一次，原地摇摆 45s；(c)`对正` 的边转边走**完全不看距离**，'
    '把人顶进 0.60m，此时人体占满画幅、`brg` 在 ±130°/±40° 乱跳。④修复：后退改缓退'
    '(−0.10m/s，`d_b` 减车体半径仍余 0.26m)、扫描按最后方位同向锁存、对正加距离门。'
    '实测同一场景：**最长完全静止 54.9s → 4.9s，后段 \\|brg\\| 中位 0°(卡死)/123°(乱跳) → 22°，'
    '距离 0.62m → 1.4~2.2m** | 完成 | M3/§15 |\n'
)

s = rd(TP)
if '| P75 |' in s and '| P76 |' in s:
    print('SKIP: P75/P76 行已存在')
else:
    lines = s.split('\n')
    idx = None
    for i, ln in enumerate(lines):
        if ln.startswith('| P74 |'):
            idx = i
            break
    if idx is None:
        print('FAIL: 找不到 P74 行')
        sys.exit(1)
    ins = []
    if '| P75 |' not in s:
        ins.append(ROW75.rstrip('\n'))
    if '| P76 |' not in s:
        ins.append(ROW76.rstrip('\n'))
    lines[idx + 1:idx + 1] = ins
    s = '\n'.join(lines)
    wr(TP, s)
    print('INSERTED rows:', len(ins))

s = rd(TP)
add = ''
if '## 15. 评测可重复性 v2' not in s:
    add += sec15
if '## 16. Gazebo 可视化' not in s:
    add += sec16
if add:
    s = s.rstrip('\n') + '\n\n' + add
    wr(TP, s)
    print('APPENDED sections')

# ---- memory
MEMTXT = """## P75/P76 可视化与徘徊根因（2026-10-26）

- **P75 评测可重复性 v2**：`person_walk` 加 `PW_RECORD` / `PW_REPLAY`，把人变成时间的
  纯函数（开环回放下发 (v,w)，不看机器人）。回放 vs 回放 mean 1.1mm / max 13.6mm，
  判定 PASS —— A/B 基准成立。关键坑：tau 必须锚**绝对仿真时间**（gz world 每次从 0 起），
  锚节点启动时刻会带 2.6s 抖动；绝对时间写入后必须在 `_load_replay` 里重基到 0。
- **P76 可视化**：本机 gz-sim 8 **没有 marker-manager 插件**，ROS Marker 传不进 Gazebo。
  可行方案：急停圈/限速圈用 chassis 上的 SDF visual（r=0.66 红盘 + r=1.60 36 段虚线圈）；
  轨迹/规划路径用 110 个 **static** 小球 + `/world/<w>/set_pose`（`gz.transport13` Python
  绑定，`Node.request(service, req, req_type, resp_type, timeout_ms)` 五个参数）。
  静态模型不进 `dynamic_pose/info`，不影响 `gt_tf` 真值链路。
- **gz GUI 坑**：GUI 在正常退出时会把布局和相机写回 `~/.gz/sim/8/gui.config`，手工改的
  `camera_pose` 每次被覆盖 ⇒ 必须在 `start_demo.sh` 里、`gz sim` 之前重写。删
  `EntityTree`/`ComponentInspector` 会让 `state=docked` 的 3D 视图塌成左下角一小块。
- **P76 根因（障碍旁徘徊）**：①`_safety_gate` 后退分支 `d_b<stop_d → v=0` 一刀切、无任何
  兜底 → 人 0.67m 在前 + 柱 0.44m 在后时冻结 54.9s；②`扫描` 写死 `w=+1.20` 与 `对正`
  按 brg 符号选向反向对打，目标在正后方时 w 每 1~3s 翻一次；③`对正` 边转边走不看距离，
  把人顶进 0.60m 导致 brg 乱跳。修复后最长静止 54.9s→4.9s，后段 |brg| 中位 123°→22°。
- `pkill -f 'gz sim'` 会匹配到**自己所在的 shell**（命令行里有这串字）⇒ 自杀，后面的
  命令全不执行。一律写成 `pkill -f '[g]z sim'`。
"""

if not os.path.exists('/dev/null'):
    pass
try:
    with sftp.file(MEM, 'r') as f:
        cur = f.read().decode('utf-8')
    if 'P75/P76 可视化与徘徊根因' in cur:
        print('SKIP: memory 已存在')
    else:
        with sftp.file(MEM, 'a') as f:
            f.write(('\n' + MEMTXT).encode('utf-8'))
        print('APPENDED memory')
except IOError:
    with sftp.file(MEM, 'w') as f:
        f.write(('# 2026-10-26\n\n' + MEMTXT).encode('utf-8'))
    print('CREATED memory')

# 校验
i, o, e = cli.exec_command(
    'cd %s && wc -l TaskPlan.md && grep -c "^| P7[56] " TaskPlan.md && '
    'grep -n "^## 15\\|^## 16" TaskPlan.md' % HOME, timeout=60)
print(o.read().decode(errors='replace'))

sftp.close()
cli.close()
print('DONE')
