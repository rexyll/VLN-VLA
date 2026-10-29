"""把 P74 行 + §14 写入 TaskPlan.md（走 VM 侧，Windows 无写权限）"""
import io
import os
import sys
import paramiko

HOME = '/home/earl/workspace/VLA'
TARGET = HOME + '/TaskPlan.md'
HERE = os.path.dirname(os.path.abspath(__file__))

P74_ROW = (
 '| P74 | 2026-10-25 | 修复+方法学 | **评测可重复性改造**（§13.6 阻断项收口）。'
 '根因：`person_walk` 与 `follower` 全程 `time.monotonic()`，而 gz 物理按仿真时间步进、'
 'RTF 有波动 —— 人的状态机时长、20Hz 定时器、`dt=0.05` 硬编码三处全落在 wall clock 上，'
 '同一 `PW_SEED` 的随机数消费序列迅速分叉，两次跑人的位置差 **11m**。'
 '而 `/clock` 早已桥接（`bridge.yml` GZ_TO_ROS）、`gt_tf`/`cmd_mux` 一直在用 `use_sim_time`，'
 '只是这两个决定轨迹的节点没用。修法：两节点各加 `_now()`（仿真时间优先、归零到启动，'
 '无 clock 回退 wall clock），`follower` 10 处 / `person_walk` 8 处 `monotonic()` 全换，'
 '`dt` 改实测 `clamp(now-prev, 0.005, 0.20)`，launch 补 `use_sim_time: True`。'
 '验证：探针确认 `use_sim_time` 下 `get_clock().now()` = 186.432s 仿真时间'
 '（注意 rclpy 日志前缀恒为 wall clock，不能用日志时间戳判断）。'
 '两轮干净重启同种子：mean|Δ| **5.57m → 0.185m**、max|Δ| **11.39m → 0.573m**、'
 '首次超 0.05m 从 0.0s 推到 14.7s，改善 30 倍但**未达 0.05m 判定线**。'
 '剩余不确定性两条：①人本身闭环（`robot_close`<0.70m 触发 ESCAPE、前方 1.3m 内转向）——'
 '**改机器人代码就不可能让人走同一条路**；②gz 按 wall clock 步进、非固定步长锁步。'
 '结论：种子+时钟只够做「回归测试」，做不了「A/B 对照」，下一步须加 `PW_REPLAY` 脚本化人 | '
 '完成（部分达成） | §14 |'
)

cli = paramiko.SSHClient()
cli.set_missing_host_key_policy(paramiko.AutoAddPolicy())
cli.connect('192.168.255.128', 22, 'earl', '111111',
            timeout=10, look_for_keys=False, allow_agent=False)
sftp = cli.open_sftp()

with sftp.file(TARGET, 'r') as f:
    s = f.read().decode('utf-8')

before = len(s)
log = []

if '| P74 |' not in s:
    # 锚定 P73 行末尾
    anchor = '无法做 A/B 归因 | 完成（无收益，仅修 bug） | M4/§12.6 |'
    n = s.count(anchor)
    if n != 1:
        print('ANCHOR_FAIL count=%d' % n)
        sys.exit(1)
    s = s.replace(anchor, anchor + '\n' + P74_ROW, 1)
    log.append('row inserted')
else:
    log.append('row exists, skip')

sec = io.open(os.path.join(HERE, 'tmp_p74_sec14.md'), encoding='utf-8').read()
if '## 14. 评测可重复性改造' not in s:
    s = s.rstrip('\n') + '\n' + sec
    log.append('section 14 appended')
else:
    log.append('section 14 exists, skip')

with sftp.file(TARGET, 'w') as f:
    f.write(s.encode('utf-8'))

with sftp.file(TARGET, 'r') as f:
    back = f.read().decode('utf-8')

print('\n'.join(log))
print('chars %d -> %d' % (before, len(back)))
print('verify P74 row:', back.count('| P74 |'))
print('verify sec14:', back.count('## 14. 评测可重复性改造'))
print('lines:', back.count('\n') + 1)
sftp.close()
cli.close()
