# -*- coding: utf-8 -*-
"""把 P73 记录行 + §13 章节写入 TaskPlan.md（幂等）。"""
import io, os, sys, re

P = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'TaskPlan.md')
s = io.open(P, encoding='utf-8').read()

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

# 1) 插 P73 行：锚定 P72 行末尾
if '| P73 |' not in s:
    anchor = '全程 pitch=0 | 完成 | M3/P69 |'
    n = s.count(anchor)
    if n != 1:
        print('ANCHOR_FAIL count=%d' % n); sys.exit(1)
    s = s.replace(anchor, anchor + '\n' + P73_ROW, 1)
    print('row inserted')
else:
    print('row exists, skip')

SEC13 = u"""
---

## 13. 距离环人速前馈（P73，2026-10-25 23:00–23:55）

### 13.1 为什么做这件事

§12.6 已定位：`person_walk` 步速 0.25~0.65 m/s（均值 ~0.45），而 follower 的 `cmd_v` 中位仅 0.113。
纯比例距离环 `v = k·(d − 1.0)` 无前馈、无积分，稳态必然停在 `d = 1.0 + v_person/k`，
实测 1.1~1.6m 与该式吻合。当时的判断是「加一项人速前馈即可消掉稳态差」。

### 13.2 做了什么

改动全部落在 `vla_follow/follower.py`（`src` / `install` / `deploy` 三份 md5 一致，最终
`119581afa309f388547baa0dc870d45d`）。

| # | 改动 | 位置 |
| --- | --- | --- |
| 1 | 新增诊断快照 `self._dbg = {ff,pid,cap,dh,df}`，TRACK 距离环出口写 `ff/pid/cap/err`，安全层写 `dh/d_f`；日志追加 `\\|ff= pid= cap= dh= df= err= vr= pv=` | `_track_step` / `_safety_gate` |
| 2 | 修 `_gap6` 未定义（必现 `NameError`）→ `geo_gap(rx, ry, _obj6)` | 真值硬保分支 |
| 3 | 新增 `fit_slope(win)` / `fit_vel(win)` 最小二乘工具 | 模块级 |
| 4 | 前馈源：2D 位置窗最小二乘 → 改标量 `robot_vfwd + d(dist)/dt`（0.7s 窗、0.5s 断窗、clamp −0.60~0.85） | TRACK |
| 5 | `_pose_cb` 新增有符号前向速度 `self.robot_vfwd`（真值位姿差分投影到航向，EWMA 0.7/0.3） | `_pose_cb` |
| 6 | `path_free` 静默停车：`if _dpath < 0.33 or not _ok6` → 只留 `_dpath < 0.33` | section 6 |
| 7 | `_safety_gate` 去掉 `elif not _okp: d_head = 0.0` 一刀切 → `d_head = min(d_f, _dpath)`；加窄道限速 `_stight < 1.00 且 v > 0.40 → v = 0.40` | `_safety_gate` |
| 8 | `path_free` 加第 4 返回值 `s_tight`，`margin` 0.10 → 0.20 | `path_free` |
| 9 | 近距离硬下界 `if dist < 0.82: v_cap = 0.0` | TRACK |

### 13.3 三次走错又回退的弯路（留档，避免复发）

1. **`_s6 < 0.30` 同时判死**：`_s6` 在「已在窄带内」时从 0.05 起算，于是机器人一旦进入窄道就自我锁死
   —— t8 出现「窄道硬停」50 次，人从 1.0m 走到 5.7m，机器人纹丝不动。**已回退**。
2. **`d_head = min(d_head, _stight + stop_d)`**：软死锁，t9 `far` 占比 63.7%。**已回退**。
3. **接近率闸** `v_cap = min(v_cap, max(0, v_ff + (dist-0.90)/0.45))`：与目的相反，
   t11 `near` 41.3%、居中 72.6%、丢失均时 27.55s。**已回退**。

### 13.4 评测结果（每轮 120s，口径统一 `track_dist_m = 1.0`）

| run | in_band | 中位 d | near | far | 居中±10° | 丢失 n | 均时 | minGap | 撞 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| t4_p72（改前基线） | **53.6%** | 1.10 | 0.0% | 46.4% | **98.2%** | 2 | 2.60 | 0.416 | 0 |
| t10_p73 | 38.1% | 1.03 | 23.7% | 38.2% | 93.1% | 2 | 3.70 | 0.454 | 0 |
| t12_p73 | 20.2% | 1.10 | 30.3% | 49.4% | 55.2% | 4 | 2.45 | 0.195 | 0 |
| t13_s7 | 39.2% | 0.97 | 36.7% | 24.1% | 78.7% | 5 | 2.94 | 0.541 | 0 |
| t15_s7b | 33.9% | 1.09 | 17.9% | 48.1% | 85.8% | 8 | 1.43 | 0.214 | 0 |

改前基线那一行全部指标最好，而且**不是**噪声能解释的（居中率 98.2% vs 55.2% 差 43 个百分点）。

### 13.5 为什么不能归因：评测本身不可重复

`person_walk` 的 `random.seed(PW_SEED)` 之外还有大量时序依赖，且主循环用 `time.time()`
而不是 `/clock`。实测：**同一个 `PW_SEED=7` 连跑两次，人的位置偏差达 11m**。
也就是说 P73 的每一轮评测走的都是不同的路，`in_band` 的 20%~39% 波动里既有代码影响也有场景差异，
无法分离。这是**方法学问题**，不是运气问题。

在此之前的六轮里还有一次基准污染：`voice_node` 播 `follow_2m.wav` 把 `target_d` 改成 2.0
（日志 `目标2.00m` 出现 111 次），已由 `tmp_run_eval.sh` 全程后台发布 `FOLLOW distance=1.0` 修正。

### 13.6 结论与下一步

**本轮没有做出可证明的改善。** 唯一确定收益是修掉 3 个真实 bug，其中 `path_free` 静默停车
（`dh` 恒 0）是**功能性缺陷**——它会在人明明在前面时让机器人原地不动，与「随时跟紧」直接冲突。
`near_pct` 也从改前的 0%（永远偏近不了，因为根本推不近）变成可调。

下一步**不建议**继续调距离环参数，而应先修评测可重复性（见 §13.7），否则任何改动都无法验证。

### 13.7 评测可重复性改造清单（建议下一步执行）

1. `person_walk` 主循环时间源 `time.time()` → `/clock`（`rclpy` 的 `now()`）；
2. 人的初始位姿、初始航向、初始速度从固定常量起（不吃 `random` 默认状态）；
3. 所有 `random` 调用收敛到单个 `random.Random(seed)` 实例，seed 全程可打印；
4. 场景侧（障碍物、机器人初始位姿）一并固定；
5. 加「回放校验」：连跑两次同 seed，人的轨迹逐点比对，`max|Δ| < 0.05m` 才算通过。

做完这五条，`in_band` / 居中率才有 A/B 意义。

### 13.8 变更文件清单

| 文件 | 变更 |
| --- | --- |
| `vla_ws/src/vla_follow/vla_follow/follower.py` | 上表 9 项全部改动；md5 `119581afa309f388547baa0dc870d45d` |
| `vla_ws/install/.../site-packages/vla_follow/follower.py` | 由 `colcon build` 安装，md5 与 src 一致（已校验） |
| `deploy/follower.py` | 同步副本，md5 与 src 一致（已校验） |
| `tmp_run_eval.sh` | 评测全程固定 `track_dist_m = 1.0`，消除演示语音污染 |
| `.stage_p0/tmp_set_follow.py` | 4Hz 持续发布 `{"action":"FOLLOW","distance":1.0}` 到 `/robot/command` |
| `tmp_p73a~f_patch.py` | 6 个幂等补丁脚本（paramiko 远程执行 + 锚点唯一性校验） |
| `tmp_p73_diag.py`、`tmp_p73_diag2.py` | 解析 `demo.log` 的 `全向限速`/`窄道硬停`/`ff=`/`pv=` 做统计 |
| `t5_p73.json` ~ `t15_s7b.json` | 11 轮评测原始数据 |
"""

if '## 13. 距离环人速前馈' not in s:
    s = s.rstrip('\n') + '\n' + SEC13
    print('section 13 appended')
else:
    print('section 13 exists, skip')

io.open(P, 'w', encoding='utf-8', newline='\n').write(s)
print('lines =', s.count('\n') + 1)
