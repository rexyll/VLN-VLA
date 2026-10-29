
### P71/P72 P0 三项落地（2026-10-25 22:00–23:10）

用户确认按核对报告的 P0 建议继续，目标"把 Nav2 打通"。结果：**打通，但端到端 GOTO 仍未达成**，卡点已从 Nav2 链路转移到差速底盘运动学标定。

**改了什么**
- 弃用 `nav2_bringup/navigation_launch.py`，新写 `launch/nav2_follow.launch.py`（只起 map_server/planner/controller/behavior/bt_navigator/velocity_smoother + lifecycle_manager）。原因：Jazzy 官方 launch 无条件拉起 10 个节点并要求全部节点都有完整参数段，缺 `collision_monitor` 一段就 `Aborting bringup`，`/navigate_to_pose` 不存在。
- `cmd_mux` 订阅 `/nav2_cmd_vel` → `/cmd_vel_smoothed`（Jazzy 的实际输出），并给 Nav2 模式 2s 宽限。
- `gt_tf.py` yaw 分母 `1-2(q.y²+q.x²)` → `1-2(q.y²+q.z²)`，抽 `quat_to_yaw()`。

**顺带修掉的必然阻断项（都实测撞到过）**
1. `follower._nav_result_cb` 读 `.result.status` → `AttributeError` → follower 进程崩溃（Nav2 一有结果就死）。改读 `fut.result().status`。
2. `cmd_mux` 硬闸遍历全部 proximity 键，把 `age`（秒，恒 <0.15）当成"贴死"→ 线速度永久清零。改为四向白名单 + 按行进方向判定。
3. `bt_navigator.default_server_timeout: 20`（**毫秒**）在本机负载下等 `follow_path` ack 常常超时 → `Goal failed`。放宽到 200ms。
4. `start_demo.sh` 与 `demo.launch.py` 各起一次 gz（双实例）→ 加 `start_gz` 参数。
5. Nav2 节点变孤儿：`pkill -x velocity_smoother` 杀不掉（comm 只有 15 字符会被截断）。改按完整路径 `pkill -f '/opt/ros/jazzy/lib/nav2_'`；新增 `vla_demo/clean_all.sh`。
6. 互斥锁被长跑子进程占用 → 陈旧锁自愈。
7. 我误把 `src/worlds/work_area.sdf`（旧 3 刚体 person）编译进 install → 已用 `deploy/work_area.sdf` 覆盖并重建。

**验收证据**：6 节点 `active [3]`、`Managed nodes are active`=1、`Aborting bringup`=0、`process has died`=0、`/navigate_to_pose` 存在、目标 accepted、controller 出控制量、mux 切 Nav2 模式、底盘真的动了。

**剩下的真正卡点（下一步）**：停掉 cmd_mux/follower 只直驱 `/robot/cmd_vel` 实测——
- `linear.x=0.30`×3s → 位移 1.385m，沿机头方向只有 0.760m（大量侧滑），幅值约 1.5×；
- `angular.z=0.80`×2s → yaw **−135.6°**（符号反、幅值约 1.5×），伴 2.34m 漂移。
→ 角速度符号相反 + 线/角幅值都偏大 ~1.5×（疑 wheel_radius/wheel_separation 与实际几何不符）+ `caster` 仍是无 joint 的游离 link。即 P69/P60a。
另：`person_walk` 在跑时人会推机器人（同指令下机头向位移 0.760m → 0.023m），做底盘标定前必须先停 `person_walk`。

**两个可复用教训**
- Windows 侧对 Z:\VLA 的**已有文件**只读（新文件可写），改动一律走 paramiko SFTP 在 VM 侧写。
- `pkill -f <pattern>` / `pgrep -f` 会自匹配执行它的那个 shell（命令里含有该 pattern），会导致"清理命令自杀、什么都没杀"。要么写进独立 .sh 文件执行，要么用 `follow[e]r` 这种括号写法。
