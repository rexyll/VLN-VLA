#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tmp_audit_taskplan.py —— 把一致性核对结果追加进 TaskPlan.md（只增不删）"""
import io
import os

P = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'TaskPlan.md')

with io.open(P, 'r', encoding='utf-8', newline='') as f:
    s = f.read()

assert 'P65' in s, 'P65 行未找到'

# ---------- 1) §4 追加 P66..P70（紧跟 P65 行） ----------
old_p65_tail = '| 进行中 | M3/P63 |'
assert s.count(old_p65_tail) == 1, 'P65 状态单元格不唯一'

p65_new = ('| 进行中（**代码已接、运行态未通，见 P67**） | M3/P63 |\n'
           '| P66 | 2026-10-25 | 变更（**补记**） | ①T6「人」躯干/头部掉落修复：`person` 模型把 '
           'skirt+torso+head 合并为单一刚体 link（质量/惯量按平行轴合成），物理上不可能再掉落 → '
           '`deploy/work_area.sdf`；②`start_demo.sh` 加 `flock` 实例互斥锁（防两次启动造成双实例污染）；'
           '③新增 Nav2 调试脚本 `tmp_nav_test.sh`/`tmp_nav_probe.sh`/`tmp_nav_probe2.sh`/`tmp_nav_manual.sh`/'
           '`tmp_tf_probe.sh`/`tmp_nuke.sh`/`tmp_p66_merge.py`/`tmp_p66_verify.sh`/`tmp_p66_body.sh` 与截图 '
           '`p66_person*.png`。该轮工作原未入档，本次核对补记 | 部分完成（修复只落在 deploy/，未进运行 world，见 P69） | M3/T6 |\n'
           '| P67 | 2026-10-25 | 问题（**阻断**） | **Nav2 从未成功启动过**（P65 声称"进行中"）。三条独立断点 + 一条致命 bug：'
           '①`nav2_params.yaml` 无 `collision_monitor` 段落 → Jazzy `navigation_launch.py` 无条件管理该节点，'
           'configure 报 `parameter \'observation_sources\' is not initialized` → `lifecycle_manager` **Aborting bringup** → '
           '`map_server/bt_navigator/collision_monitor` 停在 unconfigured、`planner/controller` inactive、'
           '**`/navigate_to_pose` action 不存在**（`Managed nodes are active` 在 demo.log 中 0 次）；'
           '②`cmd_mux` 订阅 `/nav2_cmd_vel`（pub=0），Nav2 实际输出 `/cmd_vel_smoothed`（sub=0）→ 速度链路永久不通；'
           '③`/map` pub=0（map_server 未 configure）→ 代价地图无图；'
           '④`gt_tf.py:75` 航向公式分母写成 `1-2(y²+x²)`（应为 `y²+z²`，即历史"原地打转"根因1原样复发）→ 定位航向折半。'
           '后果：follower 进 GOTO 即 `nav_active=True` + Nav2 无速度 + 0.6s 看门狗清零 = **前往人最后位置这一段无条件冻结至 12s 超时**，'
           '需求③不可实现。另：`local_costmap` 仅 static+inflation，无 obstacle 层（雷达无效，P2），Nav2 对动态障碍盲走 | open | M3/P63/P65 |\n'
           '| P68 | 2026-10-25 | 问题（**严重**） | **src 源码回退 + 三源分叉**（违反 §5 "deploy↔src 双写 md5 一致"）。'
           '①`vla_ws/src/.../follower.py`(21:17) 与 `_p58_follower.py` **逐字节相同**（`NavigateToPose` 0 处），'
           'Nav2 代码只剩 `deploy/` 与 `install/` 各一份 → 下一次 `colcon build` 即抹掉 P65；'
           '②`person_walk`(src 旧版缺 PW_SEED/P33/P38、`robot_close<1.3m` 冲突阈值、yaw 公式错)、'
           '`person_vision`(src 缺 front_ok 判据, 209 行差)、`depth_percept`(src 已改 21:16 未重建, install 仍旧版)、'
           '`work_area.sdf`(deploy 单刚体 vs src 三 link)、`nav2_params.yaml`(节点名/bond/map_server 清单)、'
           '`vision_config.json`(deploy 陈旧, aim_band 15° vs 运行 10°) 共 7 项不一致；'
           '③**根因推断（有证据）**：20:46 一次批量覆盖把本地镜像 `demo_follow/vla_ws/src`（旧版）推到权威树 `vla_ws/src`，'
           '11 个文件 mtime 同为 20:46 且内容与镜像一致；覆盖前 `work_area.sdf.bak_p61`(16:15, 42930B) 证明 src 原本用的是 42KB 世界 | open | M3/§5 |\n'
           '| P69 | 2026-10-25 | 问题 | **运行 world 缺 P60a/P61/P62/P66 全部物理修复**。运行文件 = `install/.../worlds/work_area.sdf` = '
           '`src`(`89212c5b`)：robot 的 `caster` 仍是**无 joint 游离 link**（P60a 未生效）；差速轮轴仍 `0 0 1`（P61 未生效）；'
           'DiffDrive 左右轮未对调（P62 未生效）；person 仍是 skirt/torso/head 三 link 且只挂两个轮子 joint（P66/T6 未生效）。'
           '四个 SDF（`_work_area.sdf` / `_work_area_p60.sdf` / `deploy/work_area.sdf` / `src`）**各修一部分，无一份是全量修复版**；'
           'P61/P62 只存在于根目录 `_work_area.sdf` | open | M3/P60/P61/P62/P66 |\n'
           '| P70 | 2026-10-25 | 文档 | TaskPlan 自身状态过期/漏记：①§8.2 T1「待重跑」、T3「未开始」与 §2 M3 备注及已有产物 '
           '`t1_retest.json`(17:34)/`t3_full.json`(17:38) 矛盾；②18:44–21:17 工作无条目（见 P66）；③P65 无交付物清单与阻塞记录；'
           '④G9 参数变更（`aim_band_deg` 15°→10°、新增 `use_planner/plan_*/obstacle_clear_m/person_keep_m/person_slow_m` 共 11 键）未记录；'
           '⑤P63 仍标 open，与 P65 决策冲突未标注"被取代"；⑥`demo_follow/vla_ws/src`（镜像）与 `vla_ws/src`（权威）双树并存且未定义权威/同步方向 | 已记录（待同步状态列） | TaskPlan |')

s = s.replace(old_p65_tail, p65_new, 1)

# ---------- 2) 文末追加 §10 ----------
sec10 = """

---

## 10. 代码实际进度与 TaskPlan 一致性核对（2026-10-25 21:40）

> 详细报告：`deploy/代码实际进度与TaskPlan一致性核对报告.md`。核对方式：本地文件 md5/mtime + 源码静态检查 + VM 只读探针（`ros2 lifecycle get` / `topic info` / `action list` / `demo.log` / 评测 JSON 重算）。**本轮未修改任何代码或配置。**

### 10.1 核对结论

文档记录**领先于**代码实际：P65 名义"进行中"，运行态**零生效**；P64 两项"已完成"在代码里找不到；`src` 源码树已被 P58 旧版覆盖；运行 world 缺 P60a/P61/P62/P66 全部物理修复。同时 18:44–21:17 的实际工作（P66 等）**原未入档**，本轮补记于 P66–P70。

### 10.2 逐项对照

| TaskPlan 记录 | 代码/运行实际 | 判定 |
| --- | --- | --- |
| P65 Nav2 进行中 | `/navigate_to_pose` 不存在；`bt_navigator/map_server/collision_monitor` unconfigured，`planner/controller` inactive；`Aborting bringup` 1 次、`Managed nodes are active` 0 次 | ✗ 未通 |
| P64 follower 同步 src 并重建 | `src/follower.py` = `_p58_follower.py`（md5 `fc2667be`，`NavigateToPose` 0 处）；Nav2 版只在 `deploy/`+`install/` | ✗ 已回退 |
| P64 person_walk BACK | `deploy`/`src` 两版均无 `'BACK'`；`tmp_p59_back.sh` 未落地 | ✗ 未实现 |
| P61/P62 轮轴与绑定 | 运行 SDF 仍 `0 0 1` + 未对调；仅 `_work_area.sdf` 有修复 | ✗ 未生效 |
| P60a 万向轮 joint | 运行 SDF `caster` 为无 joint 游离 link | ✗ 未生效 |
| P66/T6 person 掉落 | 运行 SDF person 仍三 link 无躯干/头 joint | ✗ 未生效（且原未入档） |
| §8.2 T1 待重跑 / T3 未开始 | 17:34 `t1_retest` 与 17:38 `t3_full` 均已产出 | ✗ 状态过期 |
| §5 deploy↔src md5 一致 | 7 个文件不一致（follower/person_walk/person_vision/depth_percept/work_area.sdf/nav2_params/vision_config） | ✗ 违反 |

### 10.3 Nav2 三条断点（P67 摘要）

1. **生命周期**：`nav2_params.yaml` 缺 `collision_monitor` → Jazzy `navigation_launch.py` 无条件纳管 → configure 失败 → 整个 manager `Aborting bringup`。
2. **速度链路**：Nav2 输出 `/cmd_vel_smoothed`（pub 1 / sub 0），`cmd_mux` 等 `/nav2_cmd_vel`（pub 0 / sub 1）。
3. **地图**：`/map` pub 0，`map_server` 未 configure。
4. **附加致命**：`gt_tf.py:75` 航向公式 `1-2(y²+x²)`（应为 `y²+z²`），定位航向折半 —— 与历史"原地打转"根因同一处错误复发。

### 10.4 需求达成度实测（重算 17:34/17:38 两份 JSON，均早于 Nav2 接入）

| 指标 | T1 定点 75s | T3 全量 150s |
| --- | --- | --- |
| 目标居中 \\|brg\\|≤10°（需求①） | **97.1%**（中位 3.1°，p90 5.2°） | **91.9%**（中位 2.2°，p90 7.7°） |
| 目标在 front 视野 | 96.6% | 79.1% |
| 四相机使用（需求②） | front 1490 / left 14 / right 10 | front 2405 / right 62 / left 44 / back 2 |
| 距离带内 0.9~1.1m | 51.3%（中位 1.075m） | 26.6%（中位 1.279m，最远 5.55m） |
| 丢失 / 最长时长 | 0 / — | 5 次 / 11.3s（GOTO 3 次到点 1 次） |
| 避碰 min_gap / 撞击 | 0.339m / 0 | 0.312m / 0 |

- 需求①（居中 ±10°）在旧构建上**已基本达标**（>90%）。
- 需求③（丢失→按最后 10 张图推测→前往→到位抓拍→途中重捕获）代码齐备，但**当前因 Nav2 未通而退化为原地扫描**。
- 距离保持是最大短板（T3 带内 26.6%）。
- **当前运行版本（20:47 起的 Nav2 构建 + 被覆盖的 src）从未被评测过。**

### 10.5 修复优先级（待确认后执行）

| 优先级 | 动作 |
| --- | --- |
| P0 | `nav2_params.yaml` 补 `collision_monitor` 段落（或从 `lifecycle_nodes` 摘除） |
| P0 | `cmd_mux` 订阅改 `/cmd_vel_smoothed`（或 launch remap 到 `/nav2_cmd_vel`） |
| P0 | `gt_tf.py:75` 分母改回 `1-2(q.y²+q.z²)` |
| P1 | 用 `deploy/` 5 文件覆盖 `vla_ws/src` → `colcon build` → 复验（含已改未编译的 `depth_percept`） |
| P1 | 定唯一权威 world SDF（建议以 `work_area.sdf.bak_p61` 42KB 为基线补 P61/P62），纯净化对照复验前进/转向 |
| P2 | 恢复 person_walk BACK + `robot_close` 改回 0.70m + 补 `PW_SEED` |
| P2 | 给 Nav2 补实时避障层（深度扇区注入 costmap 或 collision_monitor 传感器源） |
| P3 | 文档补记 + 明确 `vla_ws/src` 为唯一权威、`demo_follow/vla_ws` 只读镜像 |
"""

s = s.rstrip('\n') + sec10

TMP = P + '.new'
with io.open(TMP, 'w', encoding='utf-8', newline='') as f:
    f.write(s)

# 直接写回会被 Samba 共享的写锁拒绝，走「临时文件 + 原子替换」(带重试)
import time
last = None
for i in range(20):
    try:
        os.replace(TMP, P)
        last = None
        break
    except Exception as e:      # noqa: BLE001
        last = e
        time.sleep(0.7)
if last is not None:
    raise last
print('OK: TaskPlan.md 已追加 P66-P70 与 §10')
