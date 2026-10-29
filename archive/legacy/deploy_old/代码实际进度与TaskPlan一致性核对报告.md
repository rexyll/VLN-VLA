# 代码实际进度 vs TaskPlan.md 一致性核对报告

- 核对时间：2026-10-25 21:40–21:55
- 核对对象：`Z:\VLA`（= VM `/home/earl/workspace/VLA`，Samba 同一份）+ VM 运行态只读探针
- 判定依据：文件 md5/mtime、源码静态检查、`ros2 lifecycle get` / `ros2 topic info` / `ros2 action list`、`vla_ws/logs/demo.log`、评测 JSON 重算
- 本报告**未修改任何代码或配置**

---

## 0. 结论

**不一致，且是"文档进度 > 代码实际"的偏差。**

| TaskPlan 记录的"最新进展" | 代码实际 |
| --- | --- |
| P65「直接上 Nav2」**进行中** | Nav2 三处硬故障，**从未成功起过一次**：`/navigate_to_pose` action 不存在，全栈节点停在 unconfigured/inactive |
| P64「deploy v4 follower 同步到 vla_ws/src 并重建上线」**已完成** | 当前 `vla_ws/src/.../follower.py` 是 **P58 旧版**（md5 与 `_p58_follower.py` 完全相同，`NavigateToPose` 出现 0 次）；只有 `deploy/` 与 `install/` 里是 Nav2 版 |
| P64「person_walk 增加 BACK 后退行为(0.8~2.0s, -0.18m/s)」**已完成** | 代码里**没有 BACK**（`deploy` / `src` 两版均无 `'BACK'` 字符串）；补丁脚本 `tmp_p59_back.sh` 存在但未落地 |
| P61/P62 差速轮轴 `0 0 -1` + 左右轮绑定对调 **已完成并验证** | 只在根目录草稿 `_work_area.sdf` 里；`deploy/`、`src/worlds/`、`install/` 三处 `work_area.sdf` **仍是 `0 0 1` + 未对调** |
| P60a 万向轮 joint + 重心前移 **已部署生效** | 运行用的 world 里 `caster` 仍是**无 joint 的游离 link**，无 `caster_joint`/`caster_front_joint` |
| §8.2 T1「待重跑」/ T3「未开始」 | `t1_retest.json`（17:34）、`t3_full.json`（17:38）**都已跑完**，M3 备注也写了数字，§8.2 未同步 |
| §4 问题表止于 P65 | 18:44–21:17 还做了 **P66**（person 躯干/头部掉落修复、start_demo 实例互斥、Nav2 调试脚本），**完全没记录** |

一句话：**Nav2 方案在文档上"进行中"，在代码上"接了一半"（launch/节点/参数/地图都在，但链路三处断点），在运行态上"零生效"；而 `src` 源码树已被旧版覆盖，下一次 `colcon build` 会把 Nav2 代码整体抹掉。**

---

## 1. 阻断级问题：Nav2 从未真正跑起来（P65）

当前运行实例（`ros2 launch vla_follow demo.launch.py`，20:47 启动）实测：

### 1.1 生命周期 bringup 中止 → 导航服务不存在

```
bt_navigator:       unconfigured [1]
map_server:         unconfigured [1]
collision_monitor:  unconfigured [1]
planner_server:     inactive [2]
controller_server:  inactive [2]

ros2 action list  →  /backup /compute_path_to_pose /follow_path /spin /wait ...
                      （没有 /navigate_to_pose）
```

`demo.log` 中的直接证据：

```
[collision_monitor-19] [ERROR] parameter 'observation_sources' is not initialized
[lifecycle_manager-21] [ERROR] Failed to change state for node: collision_monitor
[lifecycle_manager-21] [ERROR] Failed to bring up all requested nodes. Aborting bringup.
```

统计：`Aborting bringup` = 1 次，`Managed nodes are active` = **0 次**，`Nav2 接管搜索` = **0 次**。

根因：Jazzy 的 `nav2_bringup/launch/navigation_launch.py` **无条件**启动 `collision_monitor` 并把它写进 `lifecycle_nodes`；而 `nav2_params.yaml` 里**没有任何 `collision_monitor:` 段落**（两份副本都没有）。一个节点 configure 失败，整个 manager 中止 → 后面所有节点都停在未激活态。

### 1.2 速度链路断：cmd_mux 订阅的话题没有发布者

```
/cmd_vel_smoothed   Publisher count: 1   Subscription count: 0   ← Nav2 实际输出
/nav2_cmd_vel       Publisher count: 0   Subscription count: 1   ← cmd_mux 在等
```

Nav2 的 `velocity_smoother` 输入被 remap 成 `/cmd_vel_nav`，输出是 `/cmd_vel_smoothed`；`cmd_mux.py` 订阅的是 `/nav2_cmd_vel`。**两边永远接不上**——即使 1.1 修好，速度也到不了底盘。

### 1.3 地图没有发布者

```
/map   Publisher count: 0   Subscription count: 2
```

`map_server` 停在 unconfigured（被 1.1 连带），全局/局部代价地图拿不到静态地图。

### 1.4 定位桥航向公式错误（致命且已踩过一次）

`vla_ws/src/vla_follow/vla_follow/gt_tf.py:75`：

```python
yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                 1.0 - 2.0 * (q.y * q.y + q.x * q.x))   # ← 分母应为 (q.y² + q.z²)
```

这正是本工程历史日志里排在第一位、曾导致"机器人+人一起倒着走/原地打转"的**同一个公式错误**（yaw=90° 会被读成 45°）。`gt_tf` 是 Nav2 唯一的定位源（`map→odom→base_link`），航向折半后 Nav2 的规划与转向必然错。

### 1.5 后果推演（当前代码路径）

`follower` 进入 GOTO 时先 `publish(/nav2_active=True)` → `cmd_mux` 切到 Nav2 分支 → Nav2 无任何速度（1.1/1.2）→ 0.6s 看门狗清零 → **机器人在"前往人最后出现位置"这一段完全冻结**，直到 `goto_timeout_s=12s` 超时转原地扫描。

即：需求"移动到目标位置再抓拍找人"目前**不可实现**，比 P63 记录的 12.3s 死锁更糟（现在是无条件冻结 12s）。

### 1.6 附带风险：Nav2 无实时避障层

`nav2_params.yaml` 的 `local_costmap` 插件只有 `["static_layer", "inflation_layer"]`，**没有 obstacle/voxel/sonar 层**，而 GPU 激光在本机无效（P2）。所以 Nav2 实质是"按静态地图盲走"：地图外的动态障碍完全看不见，唯一兜底是 `cmd_mux` 里"正前方 <0.30m 清 v"这一条硬闸——与"全程优先避障、保持设定最短距离"的要求不匹配，需要显式补传感器层或用深度扇区做 costmap 注入。

---

## 2. 严重问题：三源代码分叉 + src 已回退

### 2.1 `follower.py` 回退

| 位置 | md5(8) | 大小 | Nav2 代码 |
| --- | --- | --- | --- |
| `deploy/follower.py` | `2d245977` | 96133 | 有（`NavigateToPose` ×5） |
| `vla_ws/install/.../follower.py`（**正在跑**） | `2d245977` | 96133 | 有 |
| `vla_ws/src/.../follower.py` | `fc2667be` | 125336 | **0 处** |
| 根目录 `_p58_follower.py` | `fc2667be` | 125336 | 0 处 |

`vla_ws/src` 的 follower 与 P58 时代快照进度**逐字节相同**。mtime 21:17（晚于 20:46 的构建），说明是构建之后被旧文件覆盖的。**下次 `colcon build` 即丢失 P65 全部集成。**

### 2.2 双源（deploy ↔ src）分叉清单

TaskPlan §5 明确"改代码须 deploy↔src 双写保持 md5 一致"，实际：

| 文件 | deploy | src(`vla_ws`) | install(运行) | 差异实质 |
| --- | --- | --- | --- | --- |
| `follower.py` | `2d245977` | `fc2667be` | `2d245977` | src 回退到 P58（无 Nav2） |
| `person_walk.py` | `5065b628` | `8fcc7cc8` | `8fcc7cc8` | src 旧版：缺 `PW_SEED`、缺 P33/P38，`robot_close<1.3m`（P22 已修过的冲突阈值），**yaw 公式错** |
| `person_vision.py` | `dbc2cade` | `57f952e2` | `57f952e2` | src 旧版：缺 `front_ok`（front 优先居中）判据等 209 行差异 |
| `depth_percept.py` | `3c5431d7` | `3c5431d7` | `152c43d5` | src 已更新（21:16）但**未重建**，运行的是旧版 |
| `work_area.sdf` | `be6dd62a` | `89212c5b` | `89212c5b` | deploy 是"person 三体合并单刚体"方案，运行版是"三 link 无 joint" |
| `nav2_params.yaml` | `5ad7fda3` | `fa170bdb` | `fa170bdb` | 节点名 `lifecycle_manager` vs `lifecycle_manager_navigation`、bond 6.0 vs 8.0、map_server 清单 |
| `vision_config.json` | `98151fd0`(aim_band 15°) | — | 运行读 `vla_demo/vision_config.json`(aim_band 10°) | deploy 副本已陈旧 7 小时 |

### 2.3 20:46 的批量覆盖是分叉来源（推断，有证据支撑）

- `vla_ws/src` 下 `follower/person_walk/person_vision/depth_percept/gt_tf/cmd_mux/voice_*/bridge.yml/nav2_params.yaml/demo.launch.py/work_area.sdf` **mtime 全部为 20:46**；
- 覆盖后 4 个文件内容与本地镜像 `demo_follow/vla_ws/src` **逐字节一致**，且都是旧版；
- `vla_ws/src/vla_follow/worlds/work_area.sdf.bak_p61`（42930B，16:15）表明 16:15 时 src 用的是 42KB 世界（含 caster/torso 关节），覆盖后 src 变回 21360B 的 21KB 旧世界。

结论：**20:46 那次"同步"方向错了**，把本地旧镜像推到了权威源码树上。

---

## 3. 运行世界（SDF）缺少已记录的多项修复

运行 world = `install/vla_follow/share/vla_follow/worlds/work_area.sdf`（= src，md5 `89212c5b`）：

| 修复项 | TaskPlan 状态 | 运行 world 实际 |
| --- | --- | --- |
| P60a 万向轮 joint + 重心前移 | 已完成 | ✗ `robot/caster` 是**无 joint 的游离 link**（`work_area.sdf:324`），无 `caster_joint`/`caster_front_joint`；无 `inertial/pose 0.02` |
| P61 差速轮轴 `0 0 -1` | 已完成 | ✗ 仍为 `0 0 1`（robot 345/353、person 456/460） |
| P62 DiffDrive 左右轮绑定对调 | 已完成 | ✗ 仍为 `left_joint=left_wheel_joint`（358/359、469/470） |
| P66/T6 person 躯干+头部掉落 | **未记录** | ✗ person 仍是 `skirt/torso/head` 三 link，只有两个轮子 joint → 躯干与头是自由体 |
| P41 四相机 hfov 100°/重叠 10° | M1 已修复 | ✓ 保留 |

对照：`_work_area.sdf`（21364B）才有 `0 0 -1` + 左右轮对调；`_work_area_p60.sdf`（42934B）才有 caster/torso 关节；`deploy/work_area.sdf`（be6dd62a）只做了 person 单刚体，轮轴仍是 `0 0 1`。**四个 SDF 各修了一部分，没有任何一份是"全量修复版"。**

---

## 4. 需求达成度（重算评测 JSON，均为 Nav2 之前的旧构建）

| 指标 | T1 定点（75s，`t1_retest.json`） | T3 全量（150s，`t3_full.json`） |
| --- | --- | --- |
| 目标居中 \|brg\|≤10°（需求①） | **97.1%**（中位 3.1°，p90 5.2°） | **91.9%**（中位 2.2°，p90 7.7°） |
| 目标在 front 视野 | 96.6% | 79.1% |
| 四相机实际使用（需求②） | front 1490 / left 14 / right 10 | front 2405 / right 62 / left 44 / back 2 |
| 距离带内(0.9~1.1m) | 51.3%（中位 1.075m） | **26.6%**（中位 1.279m，最远 5.55m） |
| 目标丢失 | 0 次 | 5 次，最长 11.3s；GOTO 3 次仅到点 1 次 |
| 避碰（需求：优先避障+最短距离） | min_gap 0.339m，撞击 0 | min_gap 0.312m，撞击 0 |

要点：
- **需求①（居中 ±10°）在旧构建上已基本达标**（>90%），这是全项目最扎实的一条。
- **需求③（丢失→按最后 10 张图推测→前往→到位抓拍→途中重捕获）**：代码齐备（`_infer_target` 取窗口尾 1/3 均值+1.5s 外推、`hist` 保留 10 张），但**当前因 Nav2 未打通而无法执行**，退化为"原地扫描"。
- **距离保持**是最大短板（T3 带内 26.6%），且 105s 后距离拉到 5.5m。
- 上述数据**全部来自 17:34/17:38**，早于 20:46 的覆盖与 Nav2 接入 → **当前运行版本从未被评测过**。

---

## 5. TaskPlan 文档自身的状态问题

| # | 问题 |
| --- | --- |
| D1 | §8.2 T1「待重跑」、T3「未开始」与 §2 M3 备注里已写出的 T1/T3 数字矛盾；实际 JSON 已产出 |
| D2 | 18:44–21:17 的工作（P66 person 模型、`start_demo.sh` 互斥锁、`tmp_nuke.sh`/`tmp_tf_probe.sh`/`tmp_nav_manual.sh`、`p66_person*.png`）**完全没有条目**，§4 停在 P65 |
| D3 | P65 只写了"进行中"，**没有交付物清单**（`gt_tf.py`/`cmd_mux.py`/`gen_map.py`/`nav2_params.yaml`/`bridge.yml` 的 `/clock`/`maps/arena.pgm`），也没有记录任何一条阻塞 |
| D4 | G9 要求"感知/控制参数调整视同代码变更并记录前后值"：`aim_band_deg` 15°→10°、新增 `use_planner/plan_*/obstacle_clear_m/person_keep_m/person_slow_m` 共 11 个键（`vla_demo/vision_config.json`）无记录 |
| D5 | P63 状态仍为"open——方案已提交待用户确认"，但 P65 已决定"放弃方案 A、直接上 Nav2"，两条互相打架，P63 未标注"已被 P65 取代" |
| D6 | 目录里并存 `demo_follow/vla_ws/src`（本地镜像）与 `vla_ws/src`（权威）两套同名源码，文档没有说明谁是权威、同步方向如何——这正是 20:46 覆盖事故的土壤 |

---

## 6. 建议修复顺序（未执行，待确认）

| 优先级 | 动作 | 理由 |
| --- | --- | --- |
| **P0** | `nav2_params.yaml` 补 `collision_monitor` 段落（或从 `lifecycle_nodes` 摘除该节点） | 解除 bringup 中止，`/navigate_to_pose` 才会出现 |
| **P0** | `cmd_mux` 订阅改 `/cmd_vel_smoothed`（或 launch 里 remap 到 `/nav2_cmd_vel`） | 否则 Nav2 速度永远到不了底盘 |
| **P0** | `gt_tf.py:75` 分母改回 `1 - 2*(q.y*q.y + q.z*q.z)` | 定位航向折半，Nav2 必错 |
| **P1** | 用 `deploy/` 的 5 个文件（follower/person_walk/person_vision/work_area.sdf/nav2_params）覆盖 `vla_ws/src` → `colcon build` → 复验 | 消除 src 回退；同时把 `depth_percept`（已改未编译）一并重建 |
| **P1** | 明确唯一权威 world SDF：以 `work_area.sdf.bak_p61`(42KB，含 caster/躯干关节) 为基线，补 P61/P62 轮轴与绑定，再用"纯净化对照"复验一次前进/转向 | 现运行 world 同时缺 P60a/P61/P62/P66 |
| **P2** | 恢复 `person_walk` 的 BACK 行为 + 把 `robot_close` 阈值从 1.3m 改回 0.70m + 补 `PW_SEED` | 需求"人后退"验收前提；固定种子才能复现 |
| **P2** | 给 Nav2 补实时避障层（深度扇区注入 costmap 或 collision_monitor 的 `scan/pointcloud` 源） | 现方案对动态障碍完全盲走 |
| **P3** | 文档补记：P66、Nav2 交付物清单、T1/T3 实测、参数变更（G9）、P63 标注被 P65 取代；并明确"`vla_ws/src` 唯一权威、`demo_follow/vla_ws` 只读镜像" | 防止再次误覆盖 |

---

## 附：本次核对未做的事

- 未修改任何源码/配置/世界文件；
- 未重启或停止 VM 上的仿真（发现 `gz sim` 已自行退出、ROS 节点仍在空转，未处理）；
- 未对 Nav2 修复做验证（需先改代码并重建）。
