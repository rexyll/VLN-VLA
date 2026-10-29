
## 11. P0 执行记录与验收（2026-10-25 22:00–23:10，P71/P72）

> 本节接 §10 的修复优先级。P0 三项已全部落地并上机验证通过；打通路上又暴露出 4 个必然阻断项，一并修掉。

### 11.1 P0 三项

| 项 | 原问题 | 处理 | 验证 |
| --- | --- | --- | --- |
| ① Nav2 永不激活 | Jazzy 的 `nav2_bringup/navigation_launch.py` **无条件**拉起 10 个节点（含 smoother/route/waypoint/docking/collision_monitor）并把它们全塞进 `lifecycle_manager` 的 `node_names`；`nav2_params.yaml` 缺 `collision_monitor` 段 → configure 抛 `observation_sources is not initialized` → **Aborting bringup**，`/navigate_to_pose` 根本不存在 | 不再 include 官方 launch，新写 `launch/nav2_follow.launch.py`：只起真正需要的 6 个（map_server / planner / controller / behavior / bt_navigator / velocity_smoother）+ 一个 `lifecycle_manager`，`node_names` 显式对齐。collision_monitor 直接不启——本机 GPU 激光无效(P2)，它没有观测源，留着只能让 bringup 失败 | 6 节点全部 `active [3]`；`Managed nodes are active`=1、`Aborting bringup`=0、`process has died`=0；`ros2 action list` 出现 `/navigate_to_pose` |
| ② 速度链路前后端不匹配 | 官方 launch 把 controller 的 `cmd_vel` 重映射成 `cmd_vel_nav` 喂给 velocity_smoother，smoother 输出 `/cmd_vel_smoothed`；而 `cmd_mux` 订阅的是 `/nav2_cmd_vel`（实测 发布0/订阅1）→ 进 GOTO 后 0.6s 看门狗必然触发，速度恒 0 | `cmd_mux` 订阅改 `/cmd_vel_smoothed`；切到 Nav2 模式时给 2s 宽限（Nav2 要先规划才有速度），避免每次进 GOTO 先报一次"断流急停" | 日志里 `CMD_MUX 模式 → Nav2` 后不再持续报断流；底盘确实收到速度 |
| ③ `gt_tf` 航向被折半 | `gt_tf.py:75` 分母写成 `1-2(q.y²+q.x²)`，平面运动时分子=2wz、分母=1 → `atan2` 结果正好**折半**（60° 读成 30°），Nav2 全局规划/局部控制全走偏 | 分母改 `1-2(q.y²+q.z²)`，抽成 `quat_to_yaw()` 单点实现 | `grep` 确认 `src` 下 gt_tf / follower / person_vision / person_walk / depth_percept 五处 yaw 公式**全部**为 `y²+z²`（其中 person_walk:80、person_vision:164 原本也是错的，随 11.2 的回灌一并修好） |

### 11.2 打通路上必须顺手修的阻断项（不在原 P0 列表里）

| # | 问题 | 后果 | 处理 |
| --- | --- | --- | --- |
| ④ | `src` 源码树仍是 P58 回退版 | 一旦 `colcon build` 就把 P65 的 Nav2 集成整体抹掉 | 把 `deploy/` 的 Nav2 版 `follower.py`、以及 `deploy/` 里更完整的 `person_vision.py`(P38)、`person_walk.py`(P22/P24/P33) 回灌 `src`；旧版留在 `.bak_p0/` |
| ⑤ | `follower._nav_result_cb` 取 `fut.result().result.status` | `NavigateToPose_Result` **没有** `status` 字段 → `AttributeError` → **follower 进程崩溃退出**。表现：Nav2 一有结果就跟丢，整场演示报废 | 改为读 `fut.result().status`（动作状态码）与 `resp.result.error_code`，并整体 try/except |
| ⑥ | `cmd_mux` 硬闸遍历 `prox` 全部键 | proximity JSON 里除了 4 个方位还有 `fl/fr/bl/br` 和 **`age`（秒，恒 <0.15）** → `age` 被当成"贴死"→ 线速度**永久清零**，机器人一动不动 | 硬闸只读 `front/left/right/back` 四个键，非数值一律当未知；前进只看 front、后退只看 back、侧向不闸（否则过柱会冻住），另加"任意方向 <0.15m 直接停"兜底 |
| ⑦ | `bt_navigator` 的 `default_server_timeout: 20`（单位 ms） | 本机 4 路深度相机 + gz + ogre 同时跑，等 `follow_path` ack 常常差几毫秒 → `Timed out while waiting for action server to acknowledge` → 直接 `Goal failed` | 放宽到 200ms |
| ⑧ | `start_demo.sh` 起一次 gz，`demo.launch.py` 又起一次 | 双 Gazebo 实例抢同一批 gz topic，仿真时钟错乱甚至整体退出 | `demo.launch.py` 新增 `start_gz` 参数，`start_demo.sh` 传 `start_gz:=false` |
| ⑨ | 上一轮 Nav2 节点变孤儿 | `pkill -x velocity_smoother` 杀不掉（Linux `comm` 只有 15 字符，被截断成 `velocity_smooth`）→ 新旧两套同名节点共存，lifecycle 服务调用打到旧实例 → 随机 `get_state service is not available` / `Aborting bringup` | 新增 `vla_demo/clean_all.sh`，按**完整路径前缀** `pkill -f '/opt/ros/jazzy/lib/nav2_'` 清理；`start_demo.sh` 同步改法 |
| ⑩ | 互斥锁被长跑子进程长期占用 | demo 一跑起来就再也不能 `start_demo`（除非手动清场）；反之异常退出又会留下陈旧锁 | `start_demo.sh` 加"陈旧锁自愈"：flock 失败但确实没有 demo 在跑时，自动清锁重来 |
| ⑪ | 我误把 `src/worlds/work_area.sdf`（旧 3 刚体 person）编译进 `install` | person 的 skirt/torso/head 是三个**没有 joint** 的游离 link，物理上会散架 | 已用 `deploy/work_area.sdf`（P60a/P61 修好的单刚体 person）覆盖 `src/worlds/` 并重建；旧版留 `.bak_p0/work_area.sdf.src_old` |

### 11.3 验收结论

- **Nav2 栈：通。** 6 节点 active、`/navigate_to_pose` 存在、目标被 accept、`controller_server` 正常出控制量、`velocity_smoother` 出 `/cmd_vel_smoothed`、`cmd_mux` 切到 Nav2 模式并把速度写进 `/robot/cmd_vel`、底盘轮子真的转（机器人产生位移）。
- **"走到目标点"还没达成，但原因已不在 Nav2 链路。** 直测底盘（停掉 `cmd_mux`/follower、只留 `/robot/cmd_vel` 直驱）得到：

| 指令 | 理想 | 实测 |
| --- | --- | --- |
| `linear.x=0.30` 持续 3s | 前进 0.90m | 位移 1.385m，其中**沿机头方向只有 0.760m**（大量侧滑），量级偏大约 1.5× |
| `angular.z=0.80` 持续 2s | +91.7° | **−135.6°**（**符号反、幅值约 1.5×**），伴随 2.34m 平移漂移 |

→ 差速底盘的运动学标定是错的：**角速度符号相反**、线/角幅值都偏大约 1.5×（疑 `wheel_radius`/`wheel_separation` 与实际几何不符），并且 `caster` 仍是**没有 joint 的游离 link**（2 轮 + 1 个不起作用的支撑点 = 无法稳定直立）。这就是 §10 里的 P69/P60a，是下一步唯一阻断项。

- **另一个混淆因素已确认**：`person_walk` 在跑时，人会撞/推机器人（同一指令下"沿机头方向"的位移从 0.760m 掉到 0.023m）。做底盘标定时必须先停 `person_walk`。

### 11.4 变更文件清单（`vla_ws/src` 为唯一权威，`deploy/` 同步留档）

| 文件 | 变更 |
| --- | --- |
| `launch/nav2_follow.launch.py` | **新增**，最小化 Nav2 栈（6 节点 + lifecycle_manager） |
| `launch/demo.launch.py` | 改 include 自研 nav2 launch；新增 `start_gz` 参数 |
| `config/nav2_params.yaml` | 重写：只保留 6 个节点段；补 `bt_navigator.navigators`、`behavior_server` 完整段、costmap `robot_radius: 0.20`；`default_server_timeout` 20→200 |
| `vla_follow/gt_tf.py` | yaw 公式修正 + 抽 `quat_to_yaw()` |
| `vla_follow/cmd_mux.py` | 订阅 `/cmd_vel_smoothed`；四向硬闸白名单 + 按行进方向判定；Nav2 模式宽限 2s |
| `vla_follow/follower.py` | `_nav_result_cb` 状态码取法修正（崩溃修复） |
| `vla_follow/person_vision.py`、`person_walk.py` | 用 `deploy/` 的 P38 / P22+P24+P33 版回灌 `src` |
| `worlds/work_area.sdf` | 用 `deploy/` 的单刚体 person 版回灌 `src` |
| `vla_demo/start_demo.sh` | 重写：路径前缀杀 Nav2、陈旧锁自愈、`start_gz:=false`、单实例校验 |
| `vla_demo/clean_all.sh` | **新增**，一键清场（含 Nav2 孤儿） |
