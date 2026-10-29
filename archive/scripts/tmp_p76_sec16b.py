"""在 TaskPlan.md §16 插入 "16.7 回归评测" 并把原 16.7/16.8 顺延为 16.8/16.9"""
import paramiko

HOME = '/home/earl/workspace/VLA'
TP = HOME + '/TaskPlan.md'

SEC = """### 16.7 回归评测（回放口径，与 e1/e2 同口径）

`bash p75_eval.sh 120 t18_p76 rpl`（`walk_rec.jsonl` 开环回放）：

| run | in_band<br>0.9~1.1 m | near | far | 中位 d | min d | 最长连续<br>在带 | vis% | front% | **居中<br>≤10°** | 居中<br>中位/p90/max | 换向 n | 丢失 n | minGap |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| e1（改前） | 35.1% | 30.0% | 34.9% | 1.024 | 0.410 | 3.00 s | 94.6 | 92.4 | 98.6% | 2.2 / 5.2 / 40.0 | 6 | 2 | 0.400 |
| e2（改前） | 39.4% | 27.3% | 33.3% | 1.013 | 0.502 | 1.65 s | 85.5 | 82.3 | 99.2% | 2.2 / 5.0 / 39.1 | 8 | 3 | 0.487 |
| **t18_p76（改后）** | 33.4% | 26.4% | **40.2%** | 1.043 | **0.679** | **1.25 s** | **98.3** | **98.3** | **99.1%** | 2.0 / 5.6 / **25.0** | **0** | **0** | 0.419 |

读法（关键在**别只看 in_band**）：

1. **用户主目标「人在 front 视野中间、左右摆动 ≤10°」= 99.1%，与改前 98.6/99.2% 持平，
   且最大偏差 40°→25°、中位 2.2°→2.0°** —— 主目标无退化。
2. `in_band` 掉 1.7~6 pt **全部由 `far_pct` 上升解释**（33.3% → 40.2%），`near_pct` 反而
   更低（27.3% → 26.4%）：机器人**不再往人身上顶**，而是略站远。这正是修 1（缓退）+
   修 3（`dist < target_d−0.15` 只转不进）的预期副作用。
   `0.9~1.1 m` 是 ±10 cm 的窄带，4 cm 的均值偏移就够掉 5 pt，不足以当作退化判据。
3. 稳定性三项明显变好：`min d` 0.41→**0.679 m**（全程没再贴脸）、最长连续在带 3.00→
   **1.25 s**、换向 6/8→**0**、丢失 2/3→**0**、`front_pct` 82~92%→**98.3%**。
4. 结论：**改后没有退化**；下一轮若要把 `in_band` 拉回来，应调 `target_d` 的落脚点（让稳态
   落在 1.00 而不是 1.04），而不是退回"顶着人走"。

> 评测侧顺带补了 `center` 块（`tmp_random_eval.py: analyze()`）：直接从 `recs` 的 `brg`
> 统计 `within10_pct / within15_pct / median_deg / p90_deg / max_deg`，只取 `cam=='front'`
> 且非丢失段。原先这个最关键的指标每次都要手工从 json 现算。

"""

cli = paramiko.SSHClient()
cli.set_missing_host_key_policy(paramiko.AutoAddPolicy())
cli.connect('192.168.255.128', 22, 'earl', '111111',
            timeout=10, look_for_keys=False, allow_agent=False)
sftp = cli.open_sftp()

with sftp.file(TP, 'r') as f:
    s = f.read().decode('utf-8')

if '### 16.7 回归评测' in s:
    print('SKIP: 已存在')
else:
    # 先顺延旧编号（从后往前，避免冲突）
    s = s.replace('### 16.8 变更文件', '### 16.9 变更文件')
    s = s.replace('### 16.7 遗留（下一步）', '### 16.8 遗留（下一步）')
    s = s.replace('### 16.6 修复与实测\n', '### 16.6 修复与实测\n', 1)
    idx = s.index('### 16.8 遗留（下一步）')
    s = s[:idx] + SEC + s[idx:]
    with sftp.file(TP, 'w') as f:
        f.write(s.encode('utf-8'))
    print('INSERTED 16.7')

i, o, e = cli.exec_command(
    'cd %s && wc -l TaskPlan.md && grep -n "^### 16\\." TaskPlan.md' % HOME, timeout=60)
print(o.read().decode(errors='replace'))
sftp.close()
cli.close()
print('DONE')
