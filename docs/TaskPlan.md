# TaskPlan.md — 开发任务计划与规则（⚠ 只增不删文档）

> 本文档是本项目开发的**唯一权威流程文档**。规则见 §0，方案基线见 §1，开发计划见 §2，调试日志规范见 §3，问题记录见 §4。
> 创建：2026-10-24 ｜ 方案来源：`VLA实施方案.md` v1.0（已评审通过，按推荐决策执行）

---

## 0. 文档与开发规则

### 0.1 Guide（必须遵守的指导原则）

| 编号 | 规则 |
| --- | --- |
| G1 | 严格按 `VLA实施方案.md` v1.0 执行；阶段一 Gate-1 全绿前不启动阶段二（机械臂）开发 |
| G2 | 每次开发/调试会话必须先在 `logs/` 建立或追加日志文件（格式见 §3），无日志不操作 |
| G3 | 代码变更流程固定：本地修改 → 本地语法检查（ast）→ 推送 VM → colcon 编译 → 测试 → 记录结果 |
| G4 | 每完成/变更里程碑状态，只更新 §2 状态列与追加备注，不改任务描述本身 |
| G5 | 发现方案缺陷：在 §4 追加问题条目，原方案文本处仅追加 `[FIX-n]` 标记指向问题编号，**不删除不重写原文** |
| G6 | 一切验收以数据和日志为准；测试结果记录原始数字，不记录"大概/差不多" |
| G7 | VM（192.168.255.128, earl/111111）操作统一走 `py vm_ssh.py`（exec/put，SFTP 仅相对路径）与 `py tmp_push.py`（目录推送） |
| G8 | 二进制文件（Vosk 模型/WAV/npz/ckpt）推送后必须 md5 校验；**禁止**在 VM 上对二进制目录执行 sed 等文本处理 |
| G9 | 感知/控制参数调整视同代码变更，走 G3 流程并在日志中记录改动前后值与依据 |
| G10 | VM 侧演示启停一律用 `~/vla_demo/start_demo.sh` / `stop_demo.sh`，启动参数记录进日志 |

### 0.2 Limit（禁止事项）

| 编号 | 禁止 |
| --- | --- |
| L1 | 删除或重写 TaskPlan.md 任何已有内容（仅允许：更新 §2 状态列、全文档追加新内容、按 G5 加 `[FIX-n]` 标记） |
| L2 | 修改 `VLA实施方案.md` 已确认的验收标准数值；确需调整必须在 §4 追加变更条目说明原因，原标准保留 |
| L3 | 在 VM 上安装非 Jazzy 版本的 ROS，或改动 /opt/ros 系统目录 |
| L4 | 削弱安全层（方案 §7.2）任何规则；只允许加强 |
| L5 | 跳过阶段关卡：Gate-1 未达标不得开发机械臂（除非用户显式豁免并记入 §4） |
| L6 | 无日志记录的情况下对 VM 做状态变更操作（启停 demo、杀进程、改文件） |
| L7 | 临时脚本（tmp_*）混入正式包 vla_follow；正式包内只放交付代码 |
| L8 | 未经 md5 校验就认定二进制推送成功；未经实测（话题探针/日志）就认定传感器或节点"工作正常" |

---

## 1. 方案基线（冻结）

- 方案文档：`VLA实施方案.md` v1.0（2026-10-24 评审，用户指示"按方案实现"）。
- 决策点采纳推荐值：
  - D1 = 路径 A（深度相机不可用时：RGB + 已知地图做深度代理，显式声明降级）
  - D2 = 3 自由度机械臂（转台+大臂+小臂）+ 平行夹爪，SDF 自建
  - D3 = 实施阶段三 mini-VLA 训练闭环
  - D4 = 需要语音播报反馈（TTS）
  - D5 = 跟随距离参数 1~5m 任意（中文数字解析）
- 阶段关卡：Gate-1（方案 §4.5）→ Gate-2（方案 §5.4）→ Gate-3（mini-VLA rollout ≥ 规则版基线）。

---

## 2. 开发计划（里程碑）

> 状态取值：`未开始` `进行中` `已完成` `阻塞` `修复中`。只更新此列与备注。

| # | 里程碑 | 内容与验收 | 状态 | 备注 |
| --- | --- | --- | --- | --- |
| M0 | 环境与既有资产 | VM 环境/ROS2 Jazzy/Gazebo/语音链路/世界文件/跟随 v3 部署（历史完成，见方案 §1.1） | 已完成 | 2026-10-24 历史会话完成 |
| M1 | 深度相机排障+感知层 | 4 向深度出数且值域合理；RTF≥0.8；depth_percept 节点上线并输出 /perception/proximity 与感知误差指标 | 基本完成 | P1已修(RTF=1.00)；rgbd 彩色化+person_vision 视觉识人+视觉伺服已通(P4)；P5 深度行带几何未根治(真值兜底)；Gate-1 数值进入 M3 |
| M2 | 语音指令集扩展+NLU | 方案 §4.3 指令表全实现（含纠错表/低置信确认）；指令级识别准确率实测 | 未开始 | |
| M3 | 跟随 v3 验证收尾 | 连续 ≥3 轮 ≥90s 统计：距离命中/人停机停/零碰撞/语音响应时延 | 进行中 | 2026-10-25: deploy v4 follower 同步 src + person_walk 加 BACK(P64)。T1 定点复测通过(75s: in_band 56.1%/0.8~1.2m 90.1%/丢失≈0/中位 1.08m)；T3 全量 150s(含后退): in_band 25.4%/丢失帧 17.4%——过障视觉丢失根因已定位(P63), 修复方案待用户确认 |  |P15-P18 落地后：视觉 99%、avg 1.78m、<1.5m 58%、带宽 0.9~1.1m 命中 12%、卡死 19 次、安全余量优(障碍1.95m/墙2.86m)。带宽命中待续调优(recenter 相位耗时+跨相机距离跳变) |
| M4 | Gate-1 评审+演示 | 方案 §4.5 门禁全绿；用户现场确认 | 未开始 | 用户确认点：进入阶段二 |
| M5 | 机械臂+夹爪建模 | 3DoF 臂+平行夹爪 SDF；夹持物理实测可靠 | 未开始 | |
| M6 | 视觉抓取管线 | RGB 颜色分割+深度定位+视觉伺服对准 | 未开始 | |
| M7 | 抓放状态机+语音全流程 | GRASP/PLACE 状态机+TTS 反馈+失败恢复 | 未开始 | |
| M8 | Gate-2 评审+演示 | 方案 §5.4 门禁全绿 | 未开始 | 交付点 |
| M9 | mini-VLA 训练闭环 | 数据(LeRobot格式/归一化四件套)→训练→rollout 评估→对比规则版（Gate-3） | 未开始 | |

---

## 3. 调试日志规范

- 位置：本项目 `logs/` 目录，文件名 `M<里程碑号>_<主题>.md`（如 `M1_depth_camera.md`），同里程碑多次会话**追加**。
- 每次会话记录结构：

```markdown
## 会话 <日期 时间> — <目标一句话>
### 操作与结果
（按时间序记录：做了什么、命令/文件、观测到的原始数据）
### 结论
（达成了什么/发现了什么问题，问题登记进 TaskPlan §4）
```

- VM 状态变更（启停 demo、杀进程）前后都要留一条记录。
- 验收测试必须附：测试脚本/命令、样本数、原始指标数字。

---

## 4. 问题与变更记录表（只追加）

| # | 日期 | 类型 | 内容 | 状态 | 关联 |
| --- | --- | --- | --- | --- | --- |
| P1 | 2026-10-24 | 遗留问题 | 4 向深度相机 `/depth_*/depth_image` 无消息（方案 Q1，M1 排障中） | 已修复 | M1；根因见 logs/M1_depth_camera.md：深度图发布在基础话题 `/depth_front`（32FC1），不存在 `/depth_image` 子话题；四向实测有效+RTF=1.00 |
| P2 | 2026-10-24 | 遗留问题 | GPU 激光雷达输出全 0.12，已从控制链路移除（方案 Q2，仅保留可视化） | 已处置 | M0 |
| P3 | 2026-10-24 | 遗留问题 | 跟随 v3（前馈+P+切点绕障）已部署未完成统计验证（方案 Q4，M3 收尾） | open | M3 |
| P4 | 2026-10-24 | 用户需求变更 | ①4 相机改为 rgbd_camera（彩色 RGB+深度）；②新增相机识"人"（暗色先验+深度对齐）与视觉伺服跟随；③"人"启动逃逸先原地转向再走（修复推机器人现象）；④GUI 全局俯视视角+相机画面侧窗。原方案 §4.2/§4.4 感知与跟随定义由"真值位姿"升级为"视觉优先+真值兜底" | 处理中 | M1/M3 |
| P5 | 2026-10-24 | 问题 | rgbd 相机深度图行带几何异常（front 有效带在底部 25%，left/right/back 各不相同，疑逐相机俯仰/滚动不一致），上带裁剪 10-45% 对 side/back 无效 | open | M1 |
| P6 | 2026-10-24 | 事故 | 部署命令 `find ~/vla_ws/src ~/vla_demo -type f -exec sed` 违反 G8，再次腐蚀 Vosk 模型与 WAV 音频 → 模型加载卡死。修复：重推二进制+md5 校验；部署脚本改为仅对 *.py/*.sh/*.xml/*.sdf/*.yml/*.cfg 做 sed | 已修复 | M1 |
| P7 | 2026-10-24 | 问题 | person_vision 首帧检出面积 36k px（含地面影子）与深度 0.12m（近端伪影）；已收紧阈值+斑点上半部采样+0.3m 过滤，实测距离/面积透视关系正确（0.83m/5720px ↔ 4.82m/167px） | 已修复 | M1 |
| P8 | 2026-10-24 | 问题 | ①GUI 相机画面面板未在侧边栏显示；②Gate-1 距离命中率 30%（目标 70%），贴障爬行 0.05m/s 限速过保守、贴墙 0.12m 偏近 | ①已修复 ②调优中 | M3；面板已锚定主视图左上角(gui.config)；命中率最新 36%(<3m 92%) |
| P14 | 2026-10-24 | 问题 | 贴墙卡死：底盘边角顶墙导致无法旋转（手动 w=1.5 实测 4s yaw 0.0°），旧卡死检测仅覆盖平移 | 已修复 | M3；旋转失速检测+贴墙应急，实测 7 次脱困后恢复 |
| P15 | 2026-10-24 | 用户需求变更 | 跟踪规格改为带宽式(P15)：①目标偏出 front 相机→按设定角速度立即旋转回正；②回正后距离>1m(配置 track_dist_m, 可调)以设定速度 0.2m/s(approach_speed)靠近；③稳定保持 1m±10cm(track_dist_tol_m)。原方案 §4.4"前馈+P 保持语音指定距离(默认2m)"的默认距离改为配置 1m，语音显式给距离(如"保持2米")仍可覆盖。Gate-1 距离命中口径同步改为 0.9~1.1m | 处理中 | M3 |
| P16 | 2026-10-24 | 用户需求变更 | 角速度/线速度均加 PID（增益入配置 pid_kp/ki/kd_v/w）；跟随速度上限随人速自适应（保底 0.2，上限人速+0.25）解决"人走追不上"；人 ESCAPE 总时长看门狗 12s 强制行走 | 处理中 | M3 |
| P17 | 2026-10-24 | bug | 人动机反向走：防撞人阈值(1.2m)高于新跟踪带(1.0±0.1m)，机器人在正常跟踪位持续被命令后退。修复：阈值= target_d−tol−0.25（随跟踪距离缩放，1m 跟踪时 0.65m） | 已修复 | M3 |
| P18 | 2026-10-24 | bug | ①绕障弧推进 20Hz 连续触发致航点"逃跑"，机器人绕障碍原地打转（偏航 ±180° 翻转、w 叠加达 -3.0）；②贴墙抖动前进相位越过墙体前瞻把机器人怼进墙。修复：弧推进速率限制 ≥1.5s/次 0.4rad；抖动前进先过前瞻；w 最终总钳制 ±2.2 | 已修复 | M3；avg 距离 7.76→1.78m，<1.5m 0%→58% |
| P9 | 2026-10-24 | 问题 | 绕障切点方案缺陷：人到切点后若人仍在障碍正后方则永久停车（实测卡 7m 处 v=0） | 已修复 | M1；改为轮廓跟随（弧推进+侧别/弧进度记忆+超时换边），实测绕障5 侧别稳定 |
| P11 | 2026-10-24 | 用户需求变更 | 四相机全向识人：任意相机检出→打印所在相机+偏离中线角度；非 front 视野时先纯旋转对正到 front 居中（person_vision 四相机版+follower 对正规则已实现，注入测试通过，[视觉-left/back] 实测生效） | 处理中 | M3 |
| P12 | 2026-10-24 | 用户需求变更 | 快照式跟踪管线：①4 相机 2Hz 抓拍，仅保存含目标照片（滚动 N=10，配置文件可调，照片带相机/方位/距离标注）；②最近 N 张跨相机=横向移动→按出现顺序推测目标去向（先方位后距离）→自主导航至最后出现位置；③到位后再抓拍：在视野→对正到 front→距离>0.5m 则靠近（防再丢失）→恢复跟随 | 处理中 | M3 |
| P13 | 2026-10-24 | 问题 | 视觉假阳性系列：①障碍物阴影=暗斑（已关 cast_shadows+时间一致性滤波）；②机器人自身轮/后角在侧后相机成 0.34m 超近暗斑（已加 DEPTH_MIN=0.45 过滤+防撞人侧摆改确定式） | 已修复 | M3 |
| P63 | 2026-10-25 | 问题 | **目标经过障碍物时机器人丢失目标**（150s 评测 12 段丢失共约 24s, 17.4%）。根因三条: ①主因=GOTO 搜索段被安全层"全向急停"清零——直连路径前方余量(实测 0.61m)<lidar_stop_m(0.66m) 时 v:=0, 而绕行执行器(avoid)只在 TRACK 分支生效, 搜索段无绕行能力 → "前往受阻3s→就地扫描"→但人被障碍完全遮挡(几何: 人/障碍/机器人共线, 实测视线穿过 ob3 圆心)时扫描无效 → 12.3s 死锁(机器人位移仅 0.03m); ②人贴障/贴墙时部分遮挡+图像边缘裁剪 → 0.2~2.6s 短丢失(快照可桥接但依赖①修复); ③扫描旋转中偶获检出即触发"先原地对正"打断扫描节奏(另一次事件中反而 3.6s 重捕获, 非致命) | open——方案已提交待用户确认 | M3 |
| P64 | 2026-10-25 | 变更 | deploy v4 follower(ALIGN/TRACK 两段式+P40 调参+局部规划器) 同步到 vla_ws/src 并重建上线（此前线上跑的是旧版 src 代码, 两处 md5 不一致）；person_walk 增加 BACK 后退行为(0.8~2.0s, -0.18m/s, 倒车前预判身后危险), 满足"人随机走动/转向/后退/停顿"验收前提 | 已完成 | M3 |
| P65 | 2026-10-25 | 用户决策 | 过障丢失修复方案选型：**直接上 Nav2**（放弃先做自研绕行方案 A, 避免无用功）。范围：GOTO/搜索段由 Nav2 NavigateToPose 接管（全局规划+局部控制+恢复行为）；close-range 对正/跟踪仍用自研视觉伺服（1m 级 Nav2 不擅长）。配套：静态地图(已知障碍生成) + gt_tf 真值定位 + cmd_vel 复用器 + /clock 桥接 + use_sim_time。自研安全层保留为最终闸门（只加强不削弱, L4） | 进行中（**代码已接、运行态未通，见 P67**） | M3/P63 |
| P66 | 2026-10-25 | 变更（**补记**） | ①T6「人」躯干/头部掉落修复：`person` 模型把 skirt+torso+head 合并为单一刚体 link（质量/惯量按平行轴合成），物理上不可能再掉落 → `deploy/work_area.sdf`；②`start_demo.sh` 加 `flock` 实例互斥锁（防两次启动造成双实例污染）；③新增 Nav2 调试脚本 `tmp_nav_test.sh`/`tmp_nav_probe.sh`/`tmp_nav_probe2.sh`/`tmp_nav_manual.sh`/`tmp_tf_probe.sh`/`tmp_nuke.sh`/`tmp_p66_merge.py`/`tmp_p66_verify.sh`/`tmp_p66_body.sh` 与截图 `p66_person*.png`。该轮工作原未入档，本次核对补记 | 部分完成（修复只落在 deploy/，未进运行 world，见 P69） | M3/T6 |
| P67 | 2026-10-25 | 问题（**阻断**） | **Nav2 从未成功启动过**（P65 声称"进行中"）。三条独立断点 + 一条致命 bug：①`nav2_params.yaml` 无 `collision_monitor` 段落 → Jazzy `navigation_launch.py` 无条件管理该节点，configure 报 `parameter 'observation_sources' is not initialized` → `lifecycle_manager` **Aborting bringup** → `map_server/bt_navigator/collision_monitor` 停在 unconfigured、`planner/controller` inactive、**`/navigate_to_pose` action 不存在**（`Managed nodes are active` 在 demo.log 中 0 次）；②`cmd_mux` 订阅 `/nav2_cmd_vel`（pub=0），Nav2 实际输出 `/cmd_vel_smoothed`（sub=0）→ 速度链路永久不通；③`/map` pub=0（map_server 未 configure）→ 代价地图无图；④`gt_tf.py:75` 航向公式分母写成 `1-2(y²+x²)`（应为 `y²+z²`，即历史"原地打转"根因1原样复发）→ 定位航向折半。后果：follower 进 GOTO 即 `nav_active=True` + Nav2 无速度 + 0.6s 看门狗清零 = **前往人最后位置这一段无条件冻结至 12s 超时**，需求③不可实现。另：`local_costmap` 仅 static+inflation，无 obstacle 层（雷达无效，P2），Nav2 对动态障碍盲走 | open | M3/P63/P65 |
| P68 | 2026-10-25 | 问题（**严重**） | **src 源码回退 + 三源分叉**（违反 §5 "deploy↔src 双写 md5 一致"）。①`vla_ws/src/.../follower.py`(21:17) 与 `_p58_follower.py` **逐字节相同**（`NavigateToPose` 0 处），Nav2 代码只剩 `deploy/` 与 `install/` 各一份 → 下一次 `colcon build` 即抹掉 P65；②`person_walk`(src 旧版缺 PW_SEED/P33/P38、`robot_close<1.3m` 冲突阈值、yaw 公式错)、`person_vision`(src 缺 front_ok 判据, 209 行差)、`depth_percept`(src 已改 21:16 未重建, install 仍旧版)、`work_area.sdf`(deploy 单刚体 vs src 三 link)、`nav2_params.yaml`(节点名/bond/map_server 清单)、`vision_config.json`(deploy 陈旧, aim_band 15° vs 运行 10°) 共 7 项不一致；③**根因推断（有证据）**：20:46 一次批量覆盖把本地镜像 `demo_follow/vla_ws/src`（旧版）推到权威树 `vla_ws/src`，11 个文件 mtime 同为 20:46 且内容与镜像一致；覆盖前 `work_area.sdf.bak_p61`(16:15, 42930B) 证明 src 原本用的是 42KB 世界 | open | M3/§5 |
| P69 | 2026-10-25 | 问题 | **运行 world 缺 P60a/P61/P62/P66 全部物理修复**。运行文件 = `install/.../worlds/work_area.sdf` = `src`(`89212c5b`)：robot 的 `caster` 仍是**无 joint 游离 link**（P60a 未生效）；差速轮轴仍 `0 0 1`（P61 未生效）；DiffDrive 左右轮未对调（P62 未生效）；person 仍是 skirt/torso/head 三 link 且只挂两个轮子 joint（P66/T6 未生效）。四个 SDF（`_work_area.sdf` / `_work_area_p60.sdf` / `deploy/work_area.sdf` / `src`）**各修一部分，无一份是全量修复版**；P61/P62 只存在于根目录 `_work_area.sdf` | open | M3/P60/P61/P62/P66 |
| P70 | 2026-10-25 | 文档 | TaskPlan 自身状态过期/漏记：①§8.2 T1「待重跑」、T3「未开始」与 §2 M3 备注及已有产物 `t1_retest.json`(17:34)/`t3_full.json`(17:38) 矛盾；②18:44–21:17 工作无条目（见 P66）；③P65 无交付物清单与阻塞记录；④G9 参数变更（`aim_band_deg` 15°→10°、新增 `use_planner/plan_*/obstacle_clear_m/person_keep_m/person_slow_m` 共 11 键）未记录；⑤P63 仍标 open，与 P65 决策冲突未标注"被取代"；⑥`demo_follow/vla_ws/src`（镜像）与 `vla_ws/src`（权威）双树并存且未定义权威/同步方向 | 已记录（待同步状态列） | TaskPlan |
| P72 | 2026-10-25 | 修复+标定 | **底盘运动学标定**（P69 阻断项收口）。四处物理错误：①`caster` 原是"没有 joint 的游离 link"，补成真支撑（revolute 绕 Y + mu 0.05）；②再补**前**支撑脚 `caster_front` —— 根因是质心正好压在轮轴（x=0）上、支撑多边形 `x∈[-0.12,0]` 的**前沿就是质心**，零稳定裕度，任何扰动让它绕轮轴前倾 31°（实测四元数反算 pitch=0.5403 rad）用鼻子拖地；这才是"位移 1.385m 里 1.155m 是横滑""角速度增益 1.48×"的真因，**不是** `wheel_radius`/`wheel_separation` 错；③轮 joint `<xyz resolved_frame="child">0 0 1` → `<xyz>0 0 -1`：`resolved_frame` 不是 SDF 属性，gz 明确告警后忽略，实际轴 = 轮 link 局部 Z = 机体系 **−Y** → 正轮速 = 后退（实测命令 +0.80 得 −135.6°）；④`<axis><limits>` 元素名写错（应为 `<limit>`）被忽略。`person` 模型三处同病，一并修。标定结果：直行 **0.200 m/s**（指令 0.20，增益 **1.00×**，侧向漂移 0.000m）、原地转 **+0.50 rad/s**（符号正确、幅值精确）、全程 pitch=0 | 完成 | M3/P69 |
| P73 | 2026-10-25 | 修复+调参 | **距离环人速前馈**（§12.6 建议 1 的落地）。修掉 3 个真实 bug：①`_gap6` 全局从未赋值 → "真值硬保"分支必现 `NameError`，改 `geo_gap(rx, ry, _obj6)`；②`path_free` 静默停车 —— `_ok6` 是 1.6m 扫掠布尔量，旁边有柱子即为 False，而 `_obj6 is None` 时既不写 note 也不给转向，导致 `dh` 恒 0、指令 v 恒 0（日志实证 `dh=0.00 df=1.48` 仍停车），改为只认真实接触 `_dpath < 0.33`；③前馈**方向反了** —— 2D 位置差分受 1° 方位量化噪声污染（1.5m 处 ±0.26 m/s），人正接近（0.83→0.55m）却输出 `ff=+0.50` 全速推，把人顶到 0.41m 占满画幅致整帧漏检，改为标量观测 `robot_vfwd + d(dist)/dt`。**诚实结论：指标未改善**，`in_band` 从基线 53.6% 落到 20~39%，居中率从 98.2% 落到 55~93%；根因是**评测不可重复**（`person_walk` 用 `time.time()` 而非 `/clock`，`PW_SEED=7` 两次跑人轨迹偏差 11m），无法做 A/B 归因 | 完成（无收益，仅修 bug） | M4/§12.6 |
| P74 | 2026-10-25 | 修复+方法学 | **评测可重复性改造**（§13.6 阻断项收口）。根因：`person_walk` 与 `follower` 全程 `time.monotonic()`，而 gz 物理按仿真时间步进、RTF 有波动 —— 人的状态机时长、20Hz 定时器、`dt=0.05` 硬编码三处全落在 wall clock 上，同一 `PW_SEED` 的随机数消费序列迅速分叉，两次跑人的位置差 **11m**。而 `/clock` 早已桥接（`bridge.yml` GZ_TO_ROS）、`gt_tf`/`cmd_mux` 一直在用 `use_sim_time`，只是这两个决定轨迹的节点没用。修法：两节点各加 `_now()`（仿真时间优先、归零到启动，无 clock 回退 wall clock），`follower` 10 处 / `person_walk` 8 处 `monotonic()` 全换，`dt` 改实测 `clamp(now-prev, 0.005, 0.20)`，launch 补 `use_sim_time: True`。验证：探针确认 `use_sim_time` 下 `get_clock().now()` = 186.432s 仿真时间（注意 rclpy 日志前缀恒为 wall clock，不能用日志时间戳判断）。两轮干净重启同种子：mean|Δ| **5.57m → 0.185m**、max|Δ| **11.39m → 0.573m**、首次超 0.05m 从 0.0s 推到 14.7s，改善 30 倍但**未达 0.05m 判定线**。剩余不确定性两条：①人本身闭环（`robot_close`<0.70m 触发 ESCAPE、前方 1.3m 内转向）——**改机器人代码就不可能让人走同一条路**；②gz 按 wall clock 步进、非固定步长锁步。结论：种子+时钟只够做「回归测试」，做不了「A/B 对照」，下一步须加 `PW_REPLAY` 脚本化人 | 完成（部分达成） | §14 |
| P75 | 2026-10-26 | 方法学 | **评测可重复性 v2：开环回放**。P74 把时间源切到 `/clock` 后仍缓慢分叉，根因是**人本身闭环**（`robot_close<0.70m` 触发 ESCAPE、前方 1.5s 预测点离机器人 <1.3m 就转向）—— 改机器人代码必然改变人的路径，种子固定也没用。新增 `PW_RECORD` / `PW_REPLAY`：录制每拍实际下发的 `(t,v,w)`，回放按时间轴开环重发，状态机整段跳过。三个坑：①`tau` 锚在节点启动时刻 = 锚在抖动上（两次回放差 2.6s）→ 改锚**绝对仿真时间**；②`t` 改绝对后没重基到 0，回放要等 9s 才开始、前段整段被跳过；③手写场景文件 `t0_abs` 只能写 0，需按节点首拍兜底。结果：**回放 vs 回放 mean 0.0011m / max 0.0136m，全程从未超 0.05m，判定 PASS**（改前 0.185/0.573）；两次 120s 评测采样数完全相同（n=2442）。录制 vs 回放仍有 0.24m（物理接触耦合），故**回放口径内可做 A/B，绝对数值不可与随机行走口径混用** | 完成 | §15 |
| P76 | 2026-10-26 | 可视化+诊断 | **Gazebo 可视化 + 障碍旁徘徊根因**。①可视化：本机 gz-sim 8 **没有 marker-manager 插件**，`Marker` 传不进 Gazebo，改用「chassis 上的 SDF visual + 静态小球 set_pose」——急停圈 r=0.66 半透明红盘、限速圈 r=1.60 黄色虚线圆（36 段方块）、`gz_viz.py` 用 110 个 static 小球画实际轨迹（蓝=行进/红=停住）与 Nav2 规划路径（品红）；静态模型不进 `dynamic_pose/info`，不污染 `gt_tf` 真值链路。②诊断：新增 `/follower/diag` 10Hz JSON，导出安全层内部量；用 `PW_REPLAY` 手写受控轨迹（人绕柱 ob3 后站定 45s）复现问题。③**根因三条**：(a)`后退受阻` 一刀切 `v=0` 且无任何兜底 → 人 0.67m 在前、柱 0.44m 在后时**冻结 54.9s**；(b)`扫描` 写死 `w=+1.20` 与 `对正` 按 brg 符号选向**反向对打**，目标在正后方时 w 在+1.20/−1.50 之间每 1~3s 翻一次，原地摇摆 45s；(c)`对正` 的边转边走**完全不看距离**，把人顶进 0.60m，此时人体占满画幅、`brg` 在 ±130°/±40° 乱跳。④修复：后退改缓退(−0.10m/s，`d_b` 减车体半径仍余 0.26m)、扫描按最后方位同向锁存、对正加距离门。实测同一场景：**最长完全静止 54.9s → 4.9s，后段 \|brg\| 中位 0°(卡死)/123°(乱跳) → 22°，距离 0.62m → 1.4~2.2m** | 完成 | M3/§15 |

---

## 5. 环境速查（维护用，可追加）

- VM：192.168.255.128:22 / earl / 111111；Ubuntu 24.04；ROS 2 Jazzy（/opt/ros/jazzy）；Gazebo Harmonic 8.15（GUI=ogre1，传感器=ogre2）；Vosk 模型 `/home/earl/workspace/VLA/vla_demo/vosk-model-small-cn-0.22`
- **项目根（P20 起 canonical）**：VM `/home/earl/workspace/VLA` = Windows `Z:\VLA`（Samba workbuddy_share）；环境变量 `VLA_HOME`（默认该路径）；`export VLA_HOME=... bash $VLA_HOME/vla_demo/start_demo.sh` 工作区：VM `~/vla_ws`（colcon），本地 `demo_follow/vla_ws/src/vla_follow`
- 工作区：VM `$VLA_HOME/vla_ws`（colcon）；**deploy/ 目录为交付版代码源**（P53~P62 迭代后唯一权威），改代码须 deploy↔src 双写保持 md5 一致
- 评测入口：`cd $VLA_HOME && bash tmp_run_eval.sh <时长s> <输出名>` → `<输出名>.json` + `deploy/tools/eval_timeline.py` 分析；定点复位由评测脚本 set_pose 完成
- 运维脚本：`$VLA_HOME/vla_demo/{start_demo,stop_demo,send_voice}.sh`；日志 `$VLA_HOME/vla_ws/logs/demo.log`
- 本地工具：`py vm_ssh.py exec|put`、`py tmp_push.py <本地目录> <远程相对目录>`
- 已知坑：SFTP 仅相对路径；**sed 只允许按文本扩展名(*)且永不扫 ~/vla_demo 与音频/模型目录（G8，P6 事故两次踩中）**；colcon 需合法 maintainer 邮箱；bridge YAML 走 config_file 参数；gz Time 字段为 nsec；**rgbd_camera 话题：RGB=/depth_X/image，深度=/depth_X/depth_image**；Image 格式字段 pixel_format_type；统计消息 world_stats_pb2；`pgrep -f` 会自匹配 bash 包装串（需过滤）；**远端脚本先 rm 后 unzip 的顺序错误曾删掉待解压 zip** **；**每次测试前必须确认单实例**（`ps -ef | grep -E 'gz sim|ros_gz_bridge|vla_follo' | grep -v grep`，双实例会真值/图像跨世界串线）；**follower 20Hz 抢发 cmd_vel 会污染方向测量**，方向验证须独立 gz sim -s 或 ROS→bridge 全链路

---

## 6. 阶段成果汇总（会话 traceId `6a336e7cf72443748b83611a6812c1be` 截止）

> 截断范围：会话记录 L3282–L4077（4 个 provider 轮次区段，共 793 行）。
> 起点为用户需求「人移动到障碍物附近时机器人丢失目标、胡乱移动打转」，
> 终点为 P53–P57 根因修复报告交付（`present_files` 已执行）。

### 6.1 需求与结论

| 项 | 内容 |
| --- | --- |
| 用户需求 | 当人（目标）移动到障碍物附近时，机器人容易丢失目标，随后胡乱移动与打转，要求查根因并修复 |
| 根因定性 | **不是感知问题，是控制层三条判据在同一时刻互相打架**，最终形成极限环（limit cycle） |
| 交付物 | `deploy/近距离障碍-丢目标与打转-根因与修复.md`、`deploy/follower.py`、`deploy/tools/*`（6 个诊断工具）、3 张同种子 A/B 轨迹对比图 |

### 6.2 三条根因与修复（均已落地）

| # | 位置 | 本质 | 修复 |
| --- | --- | --- | --- |
| **P53** | `_safety_gate` 的 `d_head = min(d_f, min(d_fl,d_fr)*0.72)` | 用 8 扇区折现代理判"正前方能不能走"，把**能过的走廊判成过不去** | 改为按扇区判定，整改后空旷场心 `d_head` 由 0.245 → 1.600（放行）；ob2 正对仍停（0.350）、ob1 正前 0.60m 仍停（0.550）、ob7/ob6 斜置仍限速 |
| **P54** | `v_cap = min(max(approach_speed, v_ff), v_lat)` | 把**抗过冲安全帽当成稳态限速**，稳态时 `v_lat == 人速` 恰好平衡，跟距被钉死在 `guard_d + lat × v_p`（实测 eval15 中位 0.908m / near 48.9% / in_band 15.2%） | 改为 `v_lat = v_ff + room / lat`（限制**相对接近率**而非绝对速度），`v_cap = max(0, min(0.72, v_lat))` |
| **P55** | 后退上限 `−0.35` | 人转身朝机器人走来时后退被挡住（人前进 0.4 → 净接近 0.05 m/s），距离被一点点啃到 0.6~0.8m 再也回不来 | `err_d < −0.12` 时放开到 `−0.55`；后方安全仍由安全层 `d_b` 判据保证 |
| **P57** | `path_free` 返回值语义 | **重复扣车体半径**（正前方 1.0m 障碍被算成 `d_head=0.65`，与 `stop_d=0.66` 一比变成停在 1.0m 外）；且扫掠沿**当前**车头画直线，机器人边走边被修正 10°~30° 实际轨迹切进内侧 → 贴角卡死（实测 `PW_SEED=7` 贴 ob5 角以 0.25m 中心间隙切入，清距 0.07m，**原地转/后退两条路都堵死 → 冻死 45s，人从 1.4m 走到 5.96m**） | 拆成 `d_surf` + `tight`；补 `tight` 判据（margin 0.10→0.30m 走廊，贴角不许往前挤）；补**贴身回退**（真值清距 <8cm 且 `d_b > 0.8×stop_d` 时 `v = −0.20` 倒出来） |

### 6.3 同种子 A/B 实测效果

| 指标 | 修复前 | 修复后 |
| --- | --- | --- |
| 距离中位数 | 2.98 m | **0.96 m** |
| 全程离障碍最小余量 | — | **≥ 0.57 m** |
| 丢失持续时长（最长） | 22.9 s | **0.8 s** |

### 6.4 阶段一遗留（已转入 §7 改进方案）

- 需求原文要求的 6 项跟踪设置（front 对正 ±10°、多相机方位判定、路径可视化、障碍间距配置、人后退保距、近目标减速）**尚未实现**。
- 蓝色胶囊障碍物识别问题（P-blue）**尚未定位修复**。
- 「人」的身体与头掉落问题**尚未修复**。

---

## 7. 后续改进方案（P58 → P62 ＋ 工程化收尾）

> 对应会话记录 L4078–L5754（traceId 段之后新增内容）。

### 7.1 改进方案总览

```
用户 6 项跟踪需求(P58)
   ├─ P58a 局部规划器 LocalPlanner(A* 栅格 + 扫描线膨胀)
   ├─ P58b 路径可视化(plan_dot_* 静态球体 + UserCommands set_pose)
   ├─ P58c 障碍间距 obstacle_clear_m 默认 0.2m(可配置)
   ├─ P58d 人后退保距 / 近目标减速
   ├─ P58e front 对正 ±10° band
   ├─ P58f 四相机方位判定 → 优先转向使目标进 front
   └─ P58g 转向锁存 _turn_w(>120° 防抖)
        ↓
   S1 定点测试暴露方向异常 → 根因排查
        ↓
   P60a 物理修复(万向轮 + 重心前移 2cm)   [已完成]
   P60b 控制层鲁棒性(倾斜自恢复/深度异常/急停看门狗)  [已完成]
        ↓
   P61 差速轮 joint 轴反号修正(0 0 1 → 0 0 -1)      [已完成]
   P62 DiffDrive 左右轮绑定对调                      [已完成]
        ↓
   目录归置 + 两端同步(§8)                          [已完成]
        ↓
   ★ 剩余：S1 复测 + S2~S4 定点场景 + 完整评测 + 路径可视化验证
```

### 7.2 改进项逐条状态

#### A. 功能需求实现（P58 系列）— **代码已完成，端到端验收未完成**

| 项 | 内容 | 代码状态 | 验收状态 |
| --- | --- | --- | --- |
| 需求① | 目标在 front 相机内时边移动边对正，确保水平中心线 ±10° 内 | 已实现（band 判定 + 边走边转） | ⬜ 未验收 |
| 需求② | 目标不在 front 时，用其他 3 相机判定方位 → 优先转向使目标进 front 且居中 ±10° → 再按距离决定是否靠近，途中遇障重规划 | 已实现（`person_vision` 四相机 + `follower` ALIGN/TRACK 分支） | ⬜ 未验收 |
| 需求③ | 仿真环境画出规划路径，遇障重规划时重汇路径线 | 已实现（24 个 `plan_dot_%02d` 静态球体 + `UserCommands set_pose`；ROS 侧发布 `nav_msgs/Path` 到 `/robot/plan_path`） | ⬜ 未验证重绘 |
| 需求④ | 障碍间距默认 0.2m，配置文件可改；保持该间距后做局部规划绕行 | 已实现（`obstacle_clear_m`） | ⬜ 未验证 |
| 需求⑤ | 人后退冲向机器人时，在避障基础上保持设定距离，防碰撞 | 已实现 | ⬜ 未验收 |
| 需求⑥ | 接近目标时减速，且距离始终 ≥ 设定值 | 已实现 | ⬜ 未验收 |

#### B. 根因修复（P60/P61/P62）— **全部已完成并通过纯净化对照验证**

| 项 | 根因 | 修复 | 验证证据 |
| --- | --- | --- | --- |
| **P60a** | 机器人俯仰失稳（本体无稳定支撑） | 前后各加一个可自由滚动万向轮（revolute 绕自身轮轴）；重心前移 2cm（`inertial/pose = 0.02 0 0`） | 已部署生效 |
| **P60b** | 控制层对物理异常无感知 | 倾斜检测自恢复 + 深度异常识别 + 急停看门狗 | 已部署生效 |
| **P61** | **SDF 差速轮 joint 轴反号**：P58 起重写的 `_work_area*.sdf` 全部为 `<xyz resolved_frame="child">0 0 1</xyz>`，历史可用版 `deploy/work_area.sdf` 为 `0 0 -1` ⇒ `cmd_vel.x>0` 让机器人**沿车头反方向**移动 | 3 个 SDF 各改 4 处差速轮轴为 `0 0 -1`（caster 万向轮不动） | 纯净化对照：前进由 BAD → OK（前向点积 +1.000） |
| **P62** | **同时翻转两轮轴会连带镜像偏航**：前进 = `(v_left+v_right)/2` 是共模、偏航 ∝ `(v_right−v_left)` 是差模 ⇒ 单改轴只能得到"前进对 + 转向错" | 对调 DiffDrive 的 `left_joint`/`right_joint` 绑定（前后共模不变、差模变号） | 纯净化对照：`cmd.z=+1.5` Δyaw 由 −101.9°（右转）→ **+101.9°（左转）**；person 模型同样验证通过；ROS→bridge→gz 全链路复验：ROS x=+0.5 前向点积 **+1.389**（前进 ✓）、ROS z=+1.5 Δyaw **+100.9**（左转 ✓） |

#### C. 数据驱动的根因确认（阶段二关键证据）

| 对照项 | 历史可用版（轴 `0 0 -1`） | P58 起当前构建（轴 `0 0 1`） |
| --- | --- | --- |
| 机器人 速度·车头 | **+0.964**，同向 99.6% | **−0.143**，同向 42.9% |
| 人 移动·自身朝向 | **+0.999**，同向 100% | **−0.937**，同向 **3.1%** |
| 转向 dyaw/dt ~ bearing | **+1.450**（正确） | **−0.040**（失效） |
| 人机距离中位 | 1.53 m | **4.01 m** |

> 结论：P58 起重写 SDF 时引入的轴反号，使**机器人与人同时开始倒着走**，是"胡乱移动和打转"的物理根因。

#### D. 目录归置与两端同步（工程化收尾）— **已完成**

| 项 | 内容 | 状态 |
| --- | --- | --- |
| VM 散落文件归位 | `/home/earl/vla_ws`→`_legacy/vla_ws_home/`；`/home/earl/vla_demo`→`_legacy/vla_demo_home/`；`/home/earl/vla/code`→知识库补差异 + `_legacy/vla_home_code/`；19 个 `m1_*.sh`/`recon*.sh` 等→仓库根；`shot*.png`/`cam.rgb`/`depth_*.raw`→`shots/`；`/tmp/world_p6*.sdf` 等→`_tmp_artifacts/` | ✅ 完成（50 文件 + 2 旧副本目录） |
| 明确不迁移 | `framework.log`、`git_archive.sh`、`deploy/`、`code/`、`log/`、`logs/`（属 `cleanning_robot`/`cloud`/`nacos` 等其它项目） | ✅ 保留原位 |
| 双向同步 | 新建 `vmsync.py`（Paramiko SFTP）+ `vmsync_run.sh` | ✅ 完成 |
| 一致性校验 | 逐文件 md5：**本地 849 = VM 849，共有 849，md5 不一致 0，仅本地 0，仅 VM 0** | ✅ 完全一致 |
| 同步工具踩坑固化 | SFTP `rename` 覆盖失败 → 改「临时文件 + `exec_command` mv -f 原子替换」；Windows 需 `MSYS_NO_PATHCONV=1`；实时日志（`p60_run.log`/`traj_fov.log`/`traj_log.csv`）不参与冲突判定、末步统一拉快照；远端扫描须纳入 `.workbuddy/` 隐藏目录 | ✅ 写入 `目录归置与两端同步说明.md` |

### 7.3 改进方案中**尚未完成**的清单

| # | 待办 | 阻塞/说明 |
| --- | --- | --- |
| **T1** | **S1 定点场景复测**（机器人 (−3,0,0°)、人 (1,0)） | P62 落地后最近一次仍为「带内 0%、中位 4.31m、最小 3.05m、最大 5.09m」；19s 后机器人把目标识别到"**后相机**"并原地打转（w 在 ±1.50 间持续翻号）。已排除两因素：①P62 方向已纯净化验证通过；②**已发现 VM 上同时存在两套 `ros2 launch vla_follow` + 两个 gz server + 2 个孤儿 `parameter_bridge`**，真值与图像可能来自不同世界 → **本轮已清理**（只留 1 套 launch + 1 bridge + 1 gz server），复测待重跑 |
| **T2** | **S2 侧方 / S3 人靠近障碍 / S4 人后退冲向机器人** 四个定点场景全跑 | 未开始 |
| **T3** | **完整跟随评测**（150s，`PW_SEED` 固定种子）对比基线 eval19（in_band 46.5%、min_gap 0.289、lost.dur_max 9.8） | 未开始 |
| **T4** | **路径可视化验证**：`plan_dot_*` 随重规划重绘、`/robot/plan_path` 发布 `nav_msgs/Path`、`obstacle_clear_m=0.2` 生效 | 未开始 |
| **T5** | **蓝色胶囊障碍物识别问题**（人能穿过、机器人每次都撞） | 阶段一遗留，未定位 |
| **T6** | **「人」的身体与头掉落**（P60 前已出现） | 阶段一遗留，未修复 |
| **T7** | **交付物同步到 `deploy/`**：`follower.py`、`vision_config.json`、`work_area.sdf`（含 P61+P62）、诊断工具（`tmp_p61_axis_check.py`/`tmp_p62_person_check.py`/`tmp_p61_cmp.sh`/`tmp_an_traj.py`/`tmp_an_traj_segments.py`） | 未完成 |
| **T8** | **根因报告**（差速轮轴反号 P61 + 左右轮绑定反号 P62 + 俯仰失稳力学 + P58 需求逐条实现说明） | 未完成 |
| **T9** | **`.workbuddy/memory` 追加 P58–P62 记录** | 待补（本轮 §9 已记部分） |

---

## 8. 改进方案完成度汇总（已完成 / 未完成 一览）

### 8.1 已完成

| 分类 | 条目 | 关键结果 |
| --- | --- | --- |
| **根因修复** | P53 `d_head` 扇区判定 | 空旷场心 0.245→1.600 放行；障碍正对/斜置仍正确停车限速 |
| | P54 距离环限速语义纠正 | in_band 15.2% 起点的量级修正 |
| | P55 人后退保距放开 | `err_d < −0.12` → 后退上限 −0.55 |
| | P57 `path_free` 语义拆分 + `tight` + 贴身回退 | 消除贴角冻死 45s |
| | P60a 物理：万向轮 + 重心前移 2cm | 俯仰失稳已解决 |
| | P60b 控制层鲁棒性 | 倾斜自恢复 / 深度异常 / 急停看门狗 |
| | **P61 差速轮 joint 轴反号修正** | 前进方向 BAD→OK（前向点积 +1.000），3 个 SDF × 4 处 |
| | **P62 DiffDrive 左右轮绑定对调** | 转向 BAD→OK（Δyaw −101.9°→+101.9°），person 模型 + ROS 全链路均通过 |
| **功能实现** | 需求①front 对正 ±10° | 代码完成（验收待 T2/T3） |
| | 需求②四相机方位判定 + 优先转向 | 代码完成（验收待 T2/T3） |
| | 需求③路径可视化（`plan_dot_*` + `/robot/plan_path`） | 代码完成（验收待 T4） |
| | 需求④`obstacle_clear_m=0.2` 可配置 | 代码完成（验收待 T4） |
| | 需求⑤人后退保距 | 代码完成（验收待 T2） |
| | 需求⑥近目标减速 | 代码完成（验收待 T2） |
| **工程化** | VM 散落文件全部归位 `/home/earl/workspace/VLA` | 50 文件 + 2 旧副本目录 |
| | 两端目录双向同步 + md5 全量校验 | **本地 849 = VM 849，零差异** |
| | 同步工具与踩坑文档 | `vmsync.py`、`vmsync_run.sh`、`目录归置与两端同步说明.md` |
| **阶段一效果** | 同种子 A/B | 距离中位 2.98→0.96m；离障 ≥0.57m；丢失 22.9s→0.8s |

### 8.2 未完成

| # | 条目 | 类型 | 优先级 |
| --- | --- | --- | --- |
| T1 | S1 定点复测（环境干扰已清理，待重跑） | 验收 | 高 |
| T2 | S2/S3/S4 三个定点场景 | 验收 | 高 |
| T3 | 150s 完整评测对比基线 eval19 | 验收 | 高 |
| T4 | 路径可视化 + 障碍间距生效验证 | 验收 | 中 |
| T5 | 蓝色胶囊障碍物识别 | 功能缺陷 | 中 |
| T6 | 「人」身体与头掉落 | 功能缺陷 | 中 |
| T7 | `deploy/` 交付物同步（含 P61+P62 的 `work_area.sdf`） | 交付 | 高 |
| T8 | 根因报告（P61/P62/力学/P58 逐条） | 交付 | 高 |
| T9 | memory 追加 P58–P62 记录 | 文档 | 低 |

### 8.3 遗留风险与注意事项

1. **双实例污染是隐蔽故障源**：本轮发现 VM 上同时存在 2 套 `ros2 launch` + 2 个 gz server + 2 个孤儿 `parameter_bridge`，三者向**同一 gz-transport 总线**发布同名 topic `/world/follow_world/dynamic_pose/info`，导致真值与图像可能来自不同世界。**已清理，但每次测试前必须确认单实例**：
   ```bash
   ps -ef | grep -E 'gz sim|ros_gz_bridge|vla_follo' | grep -v grep
   # 期望：1 个 ros2 launch + 1 个 gz sim server + 1 个 parameter_bridge + 5 个业务节点
   ```
   注意 `pgrep -f` 会自匹配 bash 包装串（§5 已知坑），以 `ps` 逐行核对为准。
2. **方向验证必须走"纯净化对照"**：follower 以 20Hz 抢发 cmd_vel，会污染 gz 直连测量。正确做法是独立 `gz sim -s -r` 起世界（不经 ROS/follower），或用 ROS→bridge→gz 全链路复验。
3. **转向符号的两层耦合**：ROS 约定为 `v>0`=前进、`w>0`=左转；DiffDrive 的共模（前进）由轮轴符号决定、差模（偏航）由左右轮绑定决定 ⇒ **两者必须同时修正**，只改其一会得到"前进对+转向错"。
4. **同步工具的原子性**：该 VM 的 SFTP `rename` 覆盖会失败（`OSError: Failure`），所有上传必须走「隐藏临时文件 + `exec_command` mv -f」。
5. **`_legacy/` 与 `_tmp_artifacts/` 为归档区**：内含旧版源码与实验产物，**不参与功能验证**，但已同步到本地保持两端一致。

---

## 9. 本轮（2026-10-25 17:00）工程化收尾记录

### 9.1 目录归置（VM → `/home/earl/workspace/VLA`）

| 来源 | 去向 | 数量 |
| --- | --- | --- |
| `/home/earl/vla_ws` | `_legacy/vla_ws_home/` | 1 目录（旧版 colcon 工作区，源码全旧版） |
| `/home/earl/vla_demo` | `_legacy/vla_demo_home/` | 1 目录（旧版 demo） |
| `/home/earl/vla/code` | `VLA知识库/code/` 补差异 + `_legacy/vla_home_code/` | 1 目录（`mini_vla`/`openvla_finetune`/`test` 65MB；src 独有 `calib.json`、`gz_data.npz` 补入知识库，差异版 `train_gz.py` 存 `test/_src_variants/`） |
| `/home/earl/*.sh` | 仓库根 | 19 个（`m1_*.sh` 13 + `recon*.sh` 3 + `install.sh`/`inject.sh`/`gztest.py`/`migrate.sh`） |
| `/home/earl/vla/*.sh` | 仓库根（重名者 `_tmp_artifacts/vmhome_*`） | 16 个 |
| `/home/earl/shot*.png` `cam.rgb` `depth_*.raw` | `shots/` | 11 个 |
| `/tmp/world_p6*.sdf` `gz_*.log` `p60*.png` | `_tmp_artifacts/` | 15 个 |

### 9.2 两端同步结果

```
本地文件 849 | VM 文件 849 | 共有 849 | md5 不一致 0 | 仅本地 0 | 仅VM 0
实时日志(持续追加, 不计为不一致): p60_run.log, traj_fov.log, traj_log.csv
结果: 两端完全一致
```

冲突处置决策表（按**语义**而非单纯 mtime）：

| 文件 | 判定 | 方向 |
| --- | --- | --- |
| `_work_area.sdf` / `_work_area_p58.sdf` / `_work_area_p60.sdf` | 本地为 P61+P62 修复版，VM 为旧版 | 本地 → VM |
| `vla_ws/src/vla_follow/worlds/work_area.sdf` | VM 为 P62 交换版，本地为未交换版 | VM → 本地 |
| `tmp_demo2.sh` / `tmp_demo3.sh` | 仅 CRLF/LF 差异，内容相同 | VM → 本地（统一 LF） |
| `vmput.py` | VM 为带重试新版 | VM → 本地 |
| `p60_run.log` / `traj_fov.log` / `traj_log.csv` | VM 端实时追加 | VM → 本地（末步快照） |

### 9.3 测试环境清理（同轮附带完成）

- 终止陈旧 `ros2 launch` (pid 677310) 及其 gz sim 子树
- 终止 2 个孤儿 `parameter_bridge` (pid 654410 / 669130)
- 结果：单实例（1 launch + 1 gz server + 1 bridge + 5 业务节点），`gz topic -l` 输出单一世界 topic 集，ROS 侧 `/robot/plan_path` 等 14 个话题正常
- 移除 `/home/earl/vla` 空壳目录

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
| 目标居中 \|brg\|≤10°（需求①） | **97.1%**（中位 3.1°，p90 5.2°） | **91.9%**（中位 2.2°，p90 7.7°） |
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

## 12. 底盘运动学标定与闭环验收（2026-10-25 22:35–23:40，P72）

### 12.1 定位根因：不是参数错，是"车立不住"

§11 把 P0 三项打通后，Nav2 能下发目标、`/cmd_vel_smoothed` 有控制量，但机器人只挪 0.3m 且方向乱。当时判断是 `wheel_radius`/`wheel_separation` 与实际几何不符 —— **这个判断错了**。

真实根因用两步定位：

1. **读四元数反算姿态**（不信 `gz model -p` 的 RPY 打印，它有欧拉角分解歧义）。`/world/follow_world/pose/info` 里 robot 的四元数是 `(0.0960, 0.2490, -0.3466, 0.8993)`，代入 `pitch = asin(2(wy−zx))` 得 **0.5403 rad = 31.0°**，roll≈0。同时 `pitch` 在"刚 set_pose 复位后"是 0.0000、"行驶后"是 0.5403 —— 说明它是**跑起来才发生**的。
2. **几何验算**：质心（车体几何中心）在 `x=0`，轮轴也在 `x=0`；支撑点只有"两个轮子 + 一个**后**支撑脚 `caster`(x=−0.12)"，支撑多边形 = `x∈[−0.12, 0]`。**质心正好落在多边形前沿**，稳定裕度为 0。启动加速度/数值噪声让质心越过轮轴 → 绕轮轴前倾，一直到车头下沿触地。理论倾角 `atan(0.08/0.15)=28.1°`，实测 31°（轮子略有下沉），吻合。
3. 前倾后车头承担主要载荷、轮子失去法向力 → 打滑。这才解释了"1.385m 位移里 1.155m 是横滑"和"角速度幅值 1.48×"。

顺带查出两处**一直没生效**的写法：

| 写法 | 实际后果 |
| --- | --- |
| `<xyz resolved_frame="child">0 0 1</xyz>` | `resolved_frame` **不是 SDF 属性**，gz 告警 `XML Attribute[resolved_frame] not defined in SDF` 后忽略。轮 link 绕 X 转了 +90°，其局部 Z 轴指向机体系 **−Y** → 正轮速 = **后退**（实测命令 `angular.z=+0.80` 得 −135.6°） |
| `<axis><limits><lower>…` | 元素名应为 `<limit>`；写错后同样被忽略（本次一并改正，消掉 4 条 SDF 告警） |

### 12.2 改法

| 改动 | 内容 |
| --- | --- |
| `caster` 补 joint | `caster_joint`(revolute, 轴 `0 1 0` 绕 Y 滚动, mu 0.05 允许转向侧滑)。**注意轴不能照抄轮子的 `0 0 -1`** —— caster link 没有旋转，其 Z 轴是竖直轴，照抄会变成"绕竖轴打转"的假万向轮 |
| **前**支撑脚 | 新增 `caster_front` link + `caster_front_joint`，robot 在 `x=+0.12`、person 在 `x=+0.18`（其余同 caster）。支撑多边形变成 `x∈[−0.12,+0.12]`，质心居中、前后都有裕度 |
| 轮轴符号 | `<xyz resolved_frame="child">0 0 1</xyz>` → `<xyz>0 0 -1</xyz>`（robot/person 各 2 处）。机体系 +Y 后：正轮速 = 前进，右轮快 = 逆时针(+yaw)，与 DiffDrive 插件约定一致 |
| 元素名 | `<limits>` → `<limit>`（4 处） |
| person 同构 | person 的裙摆质心同样压在轮轴上，同病同修，否则"人"跑起来也会前倾拖地 |

真值姿态用 `/world/follow_world/pose/info` 的**四元数**直接算 roll/pitch/yaw（`gz model -p` 打印的两行 RPY 在带 pitch 时会发生欧拉角分解歧义，不能直接读）。

### 12.3 标定结果（/odom = gt_tf 真值，非插件里程计）

| 项目 | 指令 | 实测 | 判定 |
| --- | --- | --- | --- |
| 直行 | `linear.x=0.20` | **0.200 m/s**（179 点平台中位），`y` 漂移 **0.000m**，yaw 变化 0° | 增益 **1.00×** ✅ |
| 原地转 | `angular.z=+0.50` | **+0.50 rad/s**，符号 **为正**，平移漂移 0.04m | 增益 **1.00×** ✅ |
| 姿态 | — | 复位/行驶中/行驶后 pitch 均 **0.0000** | 不再前倾 ✅ |

→ **`wheel_radius`/`wheel_separation` 不需要改**，原值 0.06/0.26 与几何一致。

### 12.4 Nav2 闭环验收（不经过 follower，直接置 `/nav2_active`）

目标 `(2.50, 0.60)`，起点 `(0.31, 0.93)`：

- `Goal finished with status: SUCCEEDED`，`error_code: 0`
- 终点 `(2.316, 0.701)`，**终点误差 0.211m**（Nav2 默认 `xy_goal_tolerance` 0.25m 内）
- 路径 2.09m / 直线 2.21m，用时 12.7s
- 过程：先原地把 yaw 从 1.55 转到 ~0，再直行 2m，`cmd_mux` 日志 `CMD_MUX 模式 → Nav2` 且未触发硬闸

→ **"前往人最后出现位置"这一段现在真的能走了**，需求③的前置条件解除。

### 12.5 全量跟随评测（120s，`t4_p72.json`）

| 指标 | 结果 |
| --- | --- |
| **居中 \|brg\|≤10°（需求①）** | **98.2%**（中位 3.4°，p90 6.6°）；仅 front 相机样本 99.1% |
| \|brg\|≤5° | 75.8% |
| 可见率 / front 占比 | 93.2% / 92.4% |
| 相机使用 | front 2257、left 14、back 6、none 165（需求②的多相机切换在跑） |
| 相机切换恢复 | 3 次，均值 2.33s，最大 5.5s（跨相机 1 次 5.5s） |
| 丢失事件 | 2 次，均 2.6s，都在进 GOTO 前自行恢复 |
| 避碰 | `min_gap` **0.416m**，**撞击 0 次 / 擦碰 0 次** |
| 距离 | 中位 1.10m，`1.0±0.1` 带内 53.6%，p90 1.61m，最大 2.10m |

对比 §10 用旧数据（T3 150s）重算的历史成绩：居中率 91.9% → **98.2%**，且碰撞仍为 0。物理修好后视觉伺服的表现明显变好（以前机器人在拖地，转角和位移都不可信）。

### 12.6 剩余问题：距离滞后是控制律问题，不是物理问题

`person_walk` 的步速是 **0.25~0.65 m/s**（均值 ~0.45），而 follower 实测 `cmd_v` 中位只有 **0.113**、峰值 0.527。当人恒速 0.45 走、机器人对距离用**纯比例**控制（`v = k·(d − 1.0)`，无积分/无前馈）时，稳态必然停在 `d = 1.0 + 0.45/k`，实测 1.1~1.6m 与该式吻合。

修法（下一步建议，按性价比排序）：

1. **加速度前馈**：从 `/person/odom` 或视觉历史窗估人的速度，`v_cmd = k·(d−1.0) + v_person·cos(相对方位)` —— 一项改动即可消掉稳态差；
2. 加积分项或把比例增益提高（但会加剧 1m 级的目标点抖动，需配合死区）；
3. 需求原文的硬指标是"居中 ±10°"，该项已 98.2%，距离"跟紧"是软指标 —— 若把"跟紧"定义为 `d ≤ 1.3m`，当前 76% 左右，加了前馈后应可到 95%+。

### 12.7 变更文件清单

| 文件 | 变更 |
| --- | --- |
| `deploy/work_area.sdf`、`vla_ws/src/vla_follow/worlds/work_area.sdf` | robot/person 各补 `caster_joint` + `caster_front`(link+joint)；轮轴 `0 0 -1`；`<limits>`→`<limit>`；两文件 md5 一致（`6f585d15…` 为 v4 前状态，最终版见 install 校验） |
| `vla_ws/install/.../worlds/work_area.sdf` | 由 `colcon build` 从 src 安装，md5 与 deploy 一致（已校验，避免 §11.3 那种"build 把修复覆盖掉"的复发） |
| `.stage_p0/p72_sdf_patch.py`～`p72_sdf_patch4.py` | SDF 四轮补丁（可重放的幂等脚本） |
| `.stage_p0/p72_sample.py` | 真值位姿采样器（sim 时间戳, 20Hz+） |
| `.stage_p0/p72_calib.py` | 标定脚本：清场→归位→直行/转向→平台段增益 |
| `.stage_p0/p72_goto_test.py` | Nav2 闭环验收（置 `/nav2_active` + 下发 NavigateToPose + 终点误差） |
| `t4_p72.json`、`shots/t4_p72_traj.png` | 120s 全量评测原始数据与轨迹图 |

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

---

## 14. 评测可重复性改造（P74，2026-10-25 23:58–00:50）

### 14.1 为什么必须先做这件事

§13 的六轮评测里 `in_band` 在 20%~39% 间乱跳，而改前基线是 53.6%。当时判断是「场景不同」，
但没有证据。真正的证据在 P73 期间：**同一个 `PW_SEED=7` 连跑两次，人的位置偏差达 11m**。
评测不可重复，则任何调参都是掷骰子 —— 这是方法学问题，不是运气问题。

### 14.2 根因

`person_walk` 与 `follower` **全程用 `time.monotonic()`**，而 gz 物理按仿真时间步进、
RTF 有波动：
- 人的状态机 `state_until = monotonic() + uniform(5,9)` 落在 wall clock 上，与仿真脱耦；
- `_tick` 里 `dt = 0.05` 硬编码，定时器抖动直接变成速度积分误差；
- 20Hz 定时器在 wall clock 上跑，仿真 1 秒内实际 tick 数会变。

三处叠加后，同一随机种子的**随机数消费序列**很快就分叉（转几次向、停顿几次都变了），
轨迹自然完全不同。

关键事实：`/clock` **早就桥接好了**（`config/bridge.yml` 里 `GZ_TO_ROS`），
`gt_tf` / `cmd_mux` 也一直在用 `use_sim_time: True` —— 只是这两个真正决定轨迹的节点没用。

### 14.3 改了什么

| # | 文件 | 改动 |
| --- | --- | --- |
| 1 | `follower.py` | 新增 `self._t0_wall` / `self._t0_sim` + `_now()`：有 `/clock` 用仿真时间（归零到节点启动），否则回退 wall clock；两条分支都从 0 起，切换只跳几百 ms |
| 2 | `follower.py` | 10 处 `time.monotonic()` 全部换成 `self._now()` |
| 3 | `person_walk.py` | 同样新增 `_now()`，8 处 `time.monotonic()` 换掉 |
| 4 | `person_walk.py` | `_tick` 里 `dt = 0.05` → **实测** `dt = clamp(now - prev, 0.005, 0.20)` |
| 5 | `demo.launch.py` | `person_walk` 与 `follower` 补 `parameters=[{'use_sim_time': True}]` |

验证：探针节点在 `use_sim_time:=true` 下 `get_clock().now()` 返回 **186.432 s**（仿真时间），
`use_sim_time:=false` 下返回 `1792944459.07`（Unix 纪元）。注意 **rclpy 的日志前缀永远打
wall clock**，不能用日志时间戳判断 `use_sim_time` 是否生效 —— 实测生效时日志前缀仍是纪元值。

### 14.4 验证过程中的四个坑（都是量测工具本身错，不是被测代码错）

| 坑 | 现象 | 根因与修法 |
| --- | --- | --- |
| 1 | 录制器 `t0=463.2` 然后 `[RESET] 465.0 → 8.6` | 录制器在**上一轮 gz 还活着**时启动，抓到旧实例的 stamp（旧实例已跑 463s）。修法：录制器检测 stamp 回跳 >0.5s 即丢弃旧窗口重开 |
| 2 | 清场门禁永远是「通过」或永远是「不过」，两个方向都错 | ①`pgrep -f "gz sim"` **会匹配到自己的命令行**，计数最小值 1~2，永远判不过；②更要命的是 **`gz sim` 的 comm 是 `ruby`**（gz sim 是个 ruby 包装脚本），`pgrep -x gz` 恒为 0 —— 换成它又变成「永远通过」的假门禁，实测这一版仍在旧 gz 存活时开了录制（`[RESET] 465.0 → 8.6` 就是铁证）。正确写法：`pgrep -f "gz si[m]"`（方括号避免自匹配）或 `pgrep -x ruby` |
| 3 | 第一版比对报 `首次 >0.05m @ 0.0s`、起点差 2~3m | 按「录制首帧」对齐，但录制起点相对人的起点是随机的（演示启动耗时不定）。改用「人离出生点 >0.10m」为原点 |
| 4 | 原地转不动的 `TimeoutError` | paramiko 的 `exec_command` 等 EOF，而后台录制器持有通道 fd。改用 `setsid nohup ... </dev/null >log 2>&1` + 独立 shell 脚本 |

### 14.5 结果

两次**干净重启**、同 `PW_SEED=7`、同口径（`track_dist_m=1.0`），录制器都拿到
`t0=9.036`、`n=2779`、`dur=100.0` —— 连采样数都一模一样，说明时间基已经对齐。

| 指标 | 改造前 | 改造后 |
| --- | --- | --- |
| mean \|Δ\|（人的轨迹两轮偏差） | 5.57 m | **0.185 m** |
| max \|Δ\| | 11.39 m | **0.573 m** |
| 首次超 0.05m | 起步后 0.0s | 起步后 14.7s |
| 分段偏差 | — | 10s:0.004 / 20s:0.094 / 40s:0.171 / 60s:0.112 / 80s:0.113 |

**改善 30 倍，但未达 0.05m 的判定线。** 分段数据说明：前 10s 几乎完全一致（4mm），
之后缓慢分叉（10~20cm 量级），属于「同一条路走得略有先后」而不是「走了不同的路」。

### 14.6 剩余不确定性来自哪里（重要，决定下一步）

1. **人本身是闭环的**：`person_walk` 里有 `robot_close`（<0.70m 触发 ESCAPE）和
   「前方 1.5s 预测离机器人 <1.3m 就转向」。机器人的任何一点行为差异都会反馈到人的路径上，
   再被放大。**这意味着即使种子、时钟全固定，改机器人代码就不可能让人走同一条路。**
2. **gz 物理按 wall clock 步进**，不是固定步长锁步。两次运行的积分步长序列不同，
   物理状态在给定仿真时刻上不可能严格相等。

这两条决定了：**对齐种子+时钟只能做「回归测试」（同版本重复跑），做不了「A/B 对照」（改代码后比）。**

### 14.7 下一步（建议）

要让 A/B 有意义，必须把「人」从闭环里摘出来：

- 给 `person_walk` 加 **脚本回放模式** `PW_REPLAY=<file>`：录制一段 `(sim_t, v, w)`，
  回放时按仿真时间查表发布，**忽略机器人位置**（不再有 ESCAPE / 避让耦合）；
- 更严格的做法是把人改成 `set_pose` 逐帧位姿回放（`gz service /world/<w>/set_pose`），
  位姿完全由脚本决定，物理漂移也不再影响；
- 做完后重跑 §13.4 的表，`in_band` 的 A/B 才有意义。

### 14.8 本轮另跑的一次常规评测（`t16_p74`，120s，seed 7）

| 指标 | 值 |
| --- | --- |
| 距离 | 中位 **1.01m**（正好在目标上）、均值 1.064、`1.0±0.1` 带内 28.0%、偏近 36.4%、偏远 35.6% |
| 居中 | `front` 且 `\|brg\|≤10°` **84.1%**（占检出样本）、74.8%（占全部样本） |
| 避碰 | `min_gap` **0.695m**、撞击 0、擦碰 0（本轮避碰表现最好） |
| 丢失 | 3 次，均 **1.2s**（P73 是 2.6~3.7s），其中 1 次进了 GOTO |
| 相机切换 | 22 次，均值 1.43s，全部有响应 |
| 可见率 | 88.9%，front 占比 81.7% |

注意：**这一次不能用来和 §13.4 的表比**，口径虽同但场景仍不同（§14.6）。

### 14.9 变更文件清单

| 文件 | 变更 |
| --- | --- |
| `vla_ws/src/vla_follow/vla_follow/follower.py` | 新增 `_now()` + `_t0_wall/_t0_sim`；10 处 `time.monotonic()` → `self._now()`；md5 `cf6f626a1cf6a22be62ea564dbd7d8c0` |
| `vla_ws/src/vla_follow/vla_follow/person_walk.py` | 同上（8 处）+ `_tick` 的 `dt` 改实测；md5 `805724f88e4a04a2ac2e44975da92289` |
| `vla_ws/src/vla_follow/launch/demo.launch.py` | `person_walk` / `follower` 补 `use_sim_time: True` |
| `deploy/follower.py`、`deploy/person_walk.py` | 与 src 同步（md5 已校验一致） |
| `tmp_p74_probe.py` | 时钟探针：确认 `use_sim_time` 下拿到的是仿真时间 |
| `tmp_p74_det.py` | 人的真值轨迹录制器（按 odom 的仿真时间戳；v2 加时间基回跳检测） |
| `tmp_p74_cmp*.py` | 轨迹比对（v3 用「离出生点 0.10m」对齐） |
| `p74_rec.sh`、`p74_one.sh`、`p74_eval.sh` | 单轮录制 / 干净重启录制 / 干净重启评测（`setsid` 全重定向，避开 SSH 通道占用） |
| `tmp_p74a_patch.py` | 时间源补丁（幂等，带锚点唯一性校验） |
| `det_a.json`、`det_b.json`、`t16_p74.json` | 两轮轨迹与常规评测原始数据 |

## 15. 评测可重复性 v2：开环回放（P75，2026-10-26 09:52–10:20）

### 15.1 P74 之后仍然不够用

P74 把 `person_walk` 和 `follower` 的时间源从 wall clock 切到 `/clock` 后，同种子两轮的
轨迹最大偏差从 **11.39 m 降到 0.573 m**，方向对了，但 14.7 s 之后仍缓慢分叉。原因有两条，
第二条是死结：

1. gz 按 wall clock 步进（RTF 波动），20 Hz 定时器的抖动让 `dt` 积分漂移；
2. **人本身是闭环的** —— `robot_close`（<0.70 m）触发 ESCAPE，前方 1.5 s 预测点离机器人
   <1.3 m 就转向。也就是说**改了机器人代码，人的路径必然变**，种子固定也没用。

⇒ 只要人还在看机器人的位置做决策，任何调参都无法做 A/B 对照。

### 15.2 方案：把人变成"时间的纯函数"

在 `person_walk` 里加两个环境变量：

| 变量 | 作用 |
|---|---|
| `PW_RECORD=<file>` | 把每拍**实际下发**的 `(t, v, w)` 追加写进 jsonl（写在 `_publish` 里，保证录到的就是执行量） |
| `PW_REPLAY=<file>` | 按录制时间轴开环下发 `(v, w)`，**完全不看机器人位置/障碍/状态机** |

回放走 `_replay_tick()`，在 `_tick()` 入口分流，状态机整段跳过。查表用线性插值，超出
录制轴起点前/终点后 2 s 一律发 0，保证时间轴严格对齐。

用法：

```bash
PW_RECORD=/tmp/walk.jsonl PW_SEED=7 ros2 launch vla_follow demo.launch.py ...
PW_REPLAY=/tmp/walk.jsonl              ros2 launch vla_follow demo.launch.py ...
```

### 15.3 过程中踩到的三个坑

1. **tau 锚在"节点启动"时刻 = 锚在抖动上。** 首版 `tau` 以 `person_walk` 节点自己的启动
   时刻为零点。实测 r2 的人的起步绝对时刻是 12.7 s、r3 是 15.3 s —— 差 2.6 s。两次回放
   虽然下发同一串 `(v,w)`，却发生在世界时间轴的不同位置，机器人多跑了 2.6 s 才等到人动。
   ⇒ 改为录制**绝对仿真时间**（gz world 每次启动都从 0 起），回放用
   `tau = now_abs - hdr.t0_abs`，人的起步绝对时刻在所有轮次完全相同。
2. **绝对时间基没有重基到 0。** 把 `t` 改成绝对时间后，`_replay_tick` 仍然拿**相对** tau
   去和 `seq[0][0]`（绝对值 9.015）比较 ⇒ 要等到 tau=9.015 才开始下发，人的前 9 秒
   （含 3 s WAIT 和起步）被整段跳过。⇒ 在 `_load_replay` 里按 `hdr.t0_abs` 重基。
3. **手写的场景文件 `t0_abs` 只能写 0**（脚本无法预知节点的绝对起始时刻），而 0 会被当成
   "绝对时刻 0"⇒ 一上来就跳过整条轨迹。⇒ 加防御：`t0_abs <= 0` 时改按节点首拍锚定。

### 15.4 结果

三轮回放（录制 1 次 → 回放 2 次），录制器都拿到 `n=2779 / dur=100.0 s / resets=0`：

| 对比 | 口径 | mean \|Δ\| | max \|Δ\| | 首次 >0.05 m | 判定 |
|---|---|---|---|---|---|
| 回放 vs 回放（r7/r8） | 时间对齐 | **0.0011 m** | **0.0136 m** | **从未超过** | **PASS** |
| 回放 vs 回放（r2/r3，锚点未修） | 时间对齐 | 0.0413 m | 0.0874 m | 44.6 s | MARGINAL |
| 录制 vs 回放（r4/r7） | 时间对齐 | 0.2447 m | 0.6139 m | 32.6 s | FAIL |

口径说明：`tmp_p74_cmp3.py` 用"人离出生点 >0.10 m"对齐后按时间采样；0.10 m 阈值在
0.3 m/s 下对应 ±0.33 s 的定时量化，仅对齐误差就有 ~0.1 m。另写 `tmp_p75_cmp.py` 用
**点到折线最近距离（对称 Hausdorff）**，与时间相位无关：录制 vs 回放的路径长度是
22.00 m vs 21.98 m（差 0.02 m），横向 mean 0.040 m / max 0.166 m —— 即**同一路线、轻微横向偏移**，
而不是"走了另一条路"。

### 15.5 结论

- **回放 vs 回放已经可重复到 1.1 mm**，且连续两次 120 s 评测的采样数完全相同（`n=2442`）。
  A/B 基准成立，可以去验证调参了。
- **录制 vs 回放仍有 0.24 m**：回放只保证"命令序列相同"，但人是被物理引擎驱动的刚体，
  机器人蹭到人会把人推走；这一层耦合只能靠"逐帧 set_pose 位姿回放"彻底消除（尚未做）。
- 所以：**回放口径内的相对比较有效，绝对数值不可与随机行走口径混用。**

### 15.6 变更文件

- `vla_ws/src/vla_follow/vla_follow/person_walk.py`（+ `_now_abs` / `_load_replay` /
  `_replay_tick` / `_publish` 录制 / `main` 收文件句柄）
- `deploy/person_walk.py`
- 新增脚本：`p75_one.sh`（单轮 rec/rpl）、`p75_eval.sh`（rec/rpl 标准评测）、
  `tmp_p75_cmp.py`（Hausdorff 路径比对）、`tmp_p75_run.py`

---
## 16. Gazebo 可视化 + "障碍旁徘徊不前"根因（P76，2026-10-26 10:16–11:00）

### 16.1 需求

1. 在 Gazebo 里**实时**看到"触发机器人急停的范围"；
2. 看到**路径规划轨迹**，用于判断机器人是否按规划路径移动；
3. 查清"目标移动到障碍物附近、机器人也跟到障碍物附近时，经常在障碍物旁徘徊不前、来回摆动"
   的真实原因。

### 16.2 可视化实现

**先说为什么不用 RViz marker**：本机 gz-sim 8 的插件目录里**没有 marker-manager 插件**
（`ls /usr/lib/x86_64-linux-gnu/gz-sim-8/plugins | grep -c marker` = 0），也没有 `/marker`
服务，所以 `visualization_msgs/Marker` 传不进 Gazebo。`gz.transport13` 和 `gz.msgs10` 的
Python 绑定是齐的，但缺服务端渲染器。

最终用两条路：

#### (a) 急停圈 / 限速圈 —— 挂在 `chassis` link 上的 SDF visual

直接写进 `worlds/work_area.sdf` 的 robot `chassis` link：

| 名字 | 几何 | 含义 |
|---|---|---|
| `stop_zone_v` | 圆柱 r=**0.66**、h=0.02、红色、`transparency=0.55` | `lidar_stop_m`：障碍进入此盘即硬停 |
| `slow_ring_00..35` | 36 段 0.09×0.035×0.012 的方块拼成 r=**1.60** 虚线圆，橙色 | `lidar_slow_m`：进圈后按距离限速 |

优点：跟随零延迟、零服务调用、不占 `dynamic_pose/info`（`gt_tf` 只订阅 dynamic，所以
**静态可视化模型完全不影响真值链路**）、不影响物理（visual 无碰撞）。

#### (b) 实际轨迹 / 规划路径 —— `gz_viz` 节点 + 静态小球

新增节点 `vla_follow/gz_viz.py`：

- 启动时用 `/world/follow_world/create`（`gz.msgs.EntityFactory`）建 110 个
  **static** 小球：`trk_b_00..39`（蓝）、`trk_r_00..39`（红）、`pln_00..29`（品红）；
- `_odom_cb` 收 `/robot/odom`，每移动 **0.22 m** 用 `/world/follow_world/set_pose`
  把下一个小球挪到当前位姿；按 `/robot/cmd_vel` 的 `|v|` 选颜色：
  **蓝 = 在走（\|v\|≥0.02）、红 = 停住/被限速**；
- `_plan_cb` 收 Nav2 的 `/plan`，按 **0.30 m** 重采样到 30 个品红小球上，每次重规划刷新。

于是"走过的路"就是蓝红小球拼出来的线：蓝色段是正常行进，红色段是被卡住/限速，
一眼能看出在哪段来回摆。

#### (c) GUI 侧修正

`gz` 的 GUI 在**正常退出（SIGTERM）**时会把当前布局和相机写回 `~/.gz/sim/8/gui.config`，
手工改的 `camera_pose` 每次都被覆盖。⇒ 新增 `vla_demo/gz_gui_fix.py`，由 `start_demo.sh`
在 `gz sim` 之前调用，每次启动重写：相机 `0 0 13 0 -1.5708 0`（12 m 场地俯视）、
渲染引擎 `ogre`（与 `--render-engine ogre` 一致，不一致时回落会丢相机配置）。

> 反例留档：一度把 `EntityTree` / `ComponentInspector` 两个面板从配置里删掉想给 3D 视图
> 让宽度，结果 `state=docked` 失去停靠上下文，3D 视图**塌成左下角一小块**。已回退，
> 只留相机与引擎两处修正。

### 16.3 诊断手段：`/follower/diag`（10 Hz JSON）

0.5 s 一条的文本日志无法归因：`指令v` 和 `实际v` 是不同量纲，`note` 只在变化时才有。
新增 `_pub_diag()`，把安全层内部量按 10 Hz 打成 JSON 发布：

```
t x y yaw mode v w dist brg td note
ff pid cap dh df err dfl dfr dl dr db dp st okp av escd
```

`dfl/dfr/dl/dr/db/dp/st/okp` 在 `_safety_gate` 里一次性写进 `self._dbg`。配套
`tmp_p76_diag.py` 录成 jsonl，离线统计。

### 16.4 受控场景复现

随机回放里机器人几乎没真正进入"贴障碍"状态（`dpath` 基本饱和在 1.6，`rgap<0.35` 样本为 0），
所以用 `PW_REPLAY` **手写了一条轨迹**（`tmp_p76_scn.py`）：人从出生点 (1,0) 出发，南行 →
绕到立柱 `ob3`(0.5,−3.2, r=0.4) 的东侧 → 折向西南 → 在柱旁站定 45 s。用单轮运动学积分
校验终点与路径清距后写成 jsonl。这样每次跑的都是**完全相同的几何**。

### 16.5 根因（三条，都有数据）

#### 根因一：`后退受阻` 是一刀切硬禁，形成"前进不能、后退被堵"的永久死锁

`_safety_gate` 的后退分支原来是：

```python
elif v_t < -0.05:                       # ---- 后退 ----
    if d_b < stop_d:
        v_t = 0.0
        note = f'后退受阻 后{d_b:.2f}'
```

**只把速度钉成 0，不产生任何替代动作**：不转向、不改距离目标、不计时、不升级。对照前进
分支有"绕障 / 贴身回退 / 受阻>2.2 s→后退脱困"三级兜底，后退分支一级都没有。

死锁闭环：

1. 人在机器人正前方 **0.67 m**（比 `target_d=1.0` 近）；
2. 距离环 PID 持续给**负**速度想拉开距离；
3. 后方 **0.44 m** 是立柱 → `d_b < stop_d(0.66)` → `v=0`；
4. 误差不消失（人不动）→ PID 永远给负速度 → 永远被拒 → **永久冻结**。

实测（`diag_scn.jsonl`）：`t=31.1..86.0`，**净位移 0.05 m，冻结 54.9 秒**，363/374 拍
`note` 都是 `后退受阻 后0.44`，`v` 恒为 0.00、`w` 恒为 0.00。截图 `shot_b.png` 与之一致：
机器人正被夹在人（前 0.67 m）和绿色立柱（后 0.44 m）之间。

#### 根因二：`扫描` 与 `对正` **反向抢方向盘**

- `扫描`（视觉丢失后原地搜索）：`w_t = recenter_rot_speed * 0.8` → **写死 +1.20，永远向左**；
- `对正`（ALIGN）：`sgn = 1.0 if brg >= 0 else -1.0` → **按方位符号选向**。

目标落在正后方附近（|brg|≈145°~180°）时两者方向恰好相反：扫描往左、对正往右，
`w` 在 **+1.20 / -1.50** 之间每 1~3 s 翻一次。实测（`diag_scn2.jsonl`）机器人停在
(0.90,−4.38) 原地摇摆 **45 s**，`w` 换向 36 次（0.48 次/s），人始终 0.60 m 外没被转到。

#### 根因三：`对正` 的"边转边走"**完全不看距离**，把人顶进 0.6 m 以内

```python
if aim < math.radians(34.0):
    v_t = 0.85 * self.cfg['approach_speed']      # 固定 0.44 m/s
```

人在 1.0 m 以内时照样全速往前顶，把人压到 **0.60 m**。这个距离下人体占满 front 相机画幅，
**方位估计失效** —— 实测 `brg` 在 `-42° / +133° / +25° / -136°` 之间乱跳（同一个 0.6 m
外的人）。方位乱跳 → 对正的 `sgn` 乱翻 → 与根因二叠加，`w` 在两个方向之间来回砸。

### 16.6 修复与实测

| # | 改动 | 依据 |
|---|---|---|
| 1 | 后退分支：`d_b < stop_d` 时不硬禁，`d_b > ROBOT_R+0.10` 时允许 **−0.10 m/s 缓退**，否则才停 | `d_b=0.44` 减去车体外接半径 0.18 还有 **0.26 m 真实余量**；`stop_d` 是保守阈值，不是物理边界。10 Hz 深度、每帧只挪 1 cm |
| 2 | `扫描` 也按最后已知方位选向（锁存 8 s），与 `对正` 同向 | 消除两个控制器反向对打 |
| 3 | `对正` 的边转边走加距离门：`dist < target_d − 0.15` 时 `v_t = 0`（只转不进） | 不再把人顶到方位失效的 0.6 m |

同一受控场景、同一录制轨迹，实测对比：

| 版本 | 最长完全静止 | 后段 dist 中位 | 后段 \|brg\| 中位 | `\|w\|>0.3` 换向 | 后段 note 主因 |
|---|---|---|---|---|---|
| 修改前 | **54.9 s** | 0.67 m（恒定） | 0° | 12 次 | 后退受阻 后0.44（363 拍） |
| +修 1 | 45.5 s | 0.63 m | **123°** | 36 次（0.48/s） | 原地扫描（170 拍） |
| +修 2 | 43.2 s | 0.62 m | 123° | 32 次（0.43/s） | 原地扫描（123 拍） |
| +修 3 | **4.9 s** | 1.85 m | **22°** | 58 次（0.77/s） | 对正 −43°/−39°（小角度微调） |

读法：**修 1~3 之后，"一次卡死半分钟"消失了（54.9 s → 4.9 s），方位从"乱跳 123°"回到
"稳定 22°"，距离从"糊在 0.62 m"回到 1.4~2.2 m。**换向次数看起来变多（0.77/s），但原来的
"少"是因为机器人根本不动（`w≡0`）；现在它在持续小角度修正 —— 幅度不是 ±1.5 的砸舵，而是
±0.3 量级的修正。

### 16.7 回归评测（回放口径，与 e1/e2 同口径）

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

### 16.8 遗留（下一步）

1. **近距离方位估计不可靠是根子**：`dist < 0.7 m` 时 `brg` 方差极大，现在只靠"不准太近"
   （修 3）绕开，没有真正解决。应加方位低通 + 迟滞，或对 `dist` 分段的方位增益。
2. **`w` 的换向仍有 0.77 次/s**，需要查 `esc_dir` 的滞回在"左右余量接近"时是否仍在翻。
3. `对正` 与 `扫描` 目前是"谁先匹配谁执行"的顺序判断，缺一个显式的仲裁器（优先级 +
   最短保持时间），长期应该重构。
4. 规划路径球只在 GOTO 有 `/plan` 时出现；TRACK 模式没有规划路径，属预期。

### 16.9 变更文件

- `vla_ws/src/vla_follow/worlds/work_area.sdf`（chassis 加 `stop_zone_v` + 36 段 `slow_ring`；
  world 末尾已有 `<gui><camera>`）
- `vla_ws/src/vla_follow/vla_follow/gz_viz.py`（**新增**，110 个静态可视化小球）
- `vla_ws/src/vla_follow/setup.py`（注册 `gz_viz`）
- `vla_ws/src/vla_follow/launch/demo.launch.py`（拉起 `gz_viz`，`use_sim_time`）
- `vla_ws/src/vla_follow/vla_follow/follower.py`（`_pub_diag` 10 Hz JSON / `_dbg` 扩字段 /
  后退缓退 / 扫描同向 / 对正距离门）
- `vla_demo/gz_gui_fix.py`（**新增**）、`vla_demo/start_demo.sh`（启动前调用）
- P76b 追加：`work_area.sdf` 加静态 `overhead_cam`（`/overhead/image`）；
  `gz_gui_fix.py` 把 `Camera View` 窗口块规范化（幂等重写，顺带清掉试验用的第二个
  `ImageDisplay`）
- 诊断脚本：`tmp_p76_diag.py`、`tmp_p76_an.py`、`tmp_p76_an2.py`、`tmp_p76_an3.py`、
  `tmp_p76_scn.py`、`p76_run.sh`、`p76_scn_run.sh`
- P76c 追加：`work_area.sdf`（`overhead_cam` pitch 改 `+1.5707963`、限速圈虚线块放大）、
  `gz_viz.py`（`DOT_R` 0.085）、`gz_gui_fix.py`（删 `CameraTracking`、窗口默认
  `/overhead/image` 400×400）；新增取图工具 `tmp_gz_probe.py`（判话题是否有数据）、
  `tmp_gz_snap.py`（gz 图像话题存 PNG）、`tmp_gz_setpose.py`（运行时改模型位姿）、
  `tmp_grab_img.py`（ROS 侧取图）
- 评测脚本：`tmp_random_eval.py`（`analyze()` 新增 `center` 块：`within10_pct` / `within15_pct`
  / `median_deg` / `p90_deg` / `max_deg`，只取 `cam=='front'` 且非丢失段）

### 16.10 追加（P76b，11:00–11:20）：GUI 3D 视图会整块空白 + 顶视相机兜底

> ⚠️ **本节结论已被 §16.11 推翻**，保留是为了记录排查过程。真正的两个原因都是
> **pitch 符号写反**（camera sensor 一处、`MinimalScene camera_pose` 一处），
> 不是 VMware 虚拟显卡。下面"虚拟显卡/llvmpipe"的判断是错的，别照抄。

回归评测后想再截一张最终画面，发现 **Gazebo 的 3D 视图完全不渲染**（整块只剩 GUI 底色），
而同一窗口里的 `Camera View`（front 相机小窗）一直有图。判据很干脆：

- 服务器端渲染 OK（`front_camera` 的图正常出）⇒ 世界/模型/材质没问题；
- GUI 端 GL 不 OK ⇒ 相机配置、插件、SDF 都不是原因。

排查与证据：

| 试的 | 结果 |
|---|---|
| 重启 demo（同 ogre 配置） | 10:38 gz5、11:03 cam_ovh4 能正常渲染；10:50/10:53/10:55/11:04 整块空白 —— **间歇性** |
| `wmctrl` 缩放/激活窗口强制重绘 | 无效 |
| `ogre2` + `LIBGL_ALWAYS_SOFTWARE=1` | 同样空白 |
| `glxinfo -B` | `Vendor: VMware, Inc. / Device: SVGA3D / **Accelerated: no** / Video memory: 1MB` |

结论：**本机是 VMware 虚拟显卡 + llvmpipe 软件光栅，GUI 的 GL 视图会随上下文状态时好时坏，
不是代码问题**。`--render-engine ogre` 只是让白屏变成"底色空白"，并没有消除它。

兜底方案（不依赖 GUI 的 GL 视图）：在 `work_area.sdf` 里加一台**静态俯视相机**
`overhead_cam`（`pose 0 0 13 0 -1.5707963 0`，camera sensor 800×800、10 Hz、
topic `/overhead/image`，静态模型不进 `dynamic_pose/info`、不污染 `gt_tf`）。
`ImageDisplay` 是 Qt 贴图，不经过 3D 视图，**一定出图**。

窗口默认仍指 `/camera`（实测最稳、也是判断"人是否在 front 画面中间"最直接的窗口），
3D 视图空白时用窗口顶部的 topic 下拉切到 `/overhead/image` 即可看全场俯视。
之所以不把默认改成俯视：`ImageDisplay` 在 `/overhead/image` 上启动时**时好时坏**
（有实例能订阅上并出图，有实例下拉框空着），而 `/camera` 从来没有这个问题。

> **§16.11 修正**：上面这段的两条结论都是错的。俯视相机"时好时坏"的真因是
> **camera sensor 的 pitch 写反了**（`-90°` 是朝上看天空），与 GL、与订阅都无关；
> 修好后默认已改成 `/overhead/image`。`pose 0 0 13 0 -1.5707963 0` 这个值也是错的，
> 必须 `+1.5707963`。

踩到的两个坑：

1. **不能挂两个 `ImageDisplay`**：第二个会抢 image provider，日志里是
   `QML Image: Invalid image provider: image://plugin…imagedisplay/…`，窗口空白且
   `gz topic -i` 显示 `/overhead/image` **No subscribers**。只保留一个、改它的 topic。
2. **`gui.config` 是扁平结构**（`<dialog>`/`<window>`/`<plugin>` 多个顶层元素，**没有**
   `<gui>` 根节点），按 XML 单根去 parse 会报 `junk after document element`，
   也不能用 `</gui>` 定位插入点，只能追加到文件末尾。

> 运维坑（又踩一次）：`pkill -f 'gz sim'` 与 `gz sim …` 启动命令写在**同一条**远端命令行里
> 时，命令行自身就含 `gz sim` 字样，pkill 会把自己的 shell 一起杀掉，表现为「命令静默返回、
> 什么都没执行」。要么拆成两次调用，要么写成脚本文件再 `bash 脚本`（脚本的 cmdline 只有路径）。

### 16.11 追加（P76c，11:12–11:30）：两处"空白"都是 pitch 符号写反，不是 GL 坏了

`Z:` 共享盘恢复后重新验证，发现 §16.10 的两条判断都要推翻。这次没有停在"虚拟显卡玄学"上，
而是把**服务器端相机的原始帧**直接取出来看：

```
gz topic -i -t /overhead/image   → 有 publisher(10Hz)，但 No subscribers
gz topic -i -t /camera           → 有 publisher，也有 subscriber
用 gz.transport13 订阅 /overhead/image → 6s 收到 60 帧 800×800 RGB，数据量 1920000
把首帧写 PNG → 仅 3692 字节，整幅 **纯天蓝**，一个像素的几何都没有
```

纯天蓝 = 天空色。也就是说**相机在朝上看**，不是渲染失败。原因：

> `<sensor type="camera">` 沿自身 **+X** 看。绕 Y 转 `pitch = -90°` 会把 +X 转到 **+Z（朝上）**，
> 于是拍到的是天空；俯视必须写 `pitch = +1.5707963`（+X → −Z，朝下）。
> 这和 GUI 的 `<camera_pose>`（同样是 `0 0 13 0 -1.5708 0` 却是标准俯视）**符号约定相反**——
> 我一开始照抄了 GUI 的值，于是"顶视相机没图"被误归因成 VMware GL 问题，白查了一轮。

验证（不动 SDF，直接改运行时位姿再拍）：

| overhead_cam 的 pitch | 帧内容 | 判定 |
|---|---|---|
| `-1.5708`（原值） | PNG 3692 B，**1 种颜色** | 只有天空 |
| `+1.5708`（`set_pose` 热改） | PNG 有 **1179 种颜色** | 地板/墙/障碍/机器人/急停盘/限速圈全在 |

三处配套修正：

1. **`work_area.sdf`**：`overhead_cam` 的 `pose` 改 `0 0 13 0 1.5707963 0`，并把上面这条
   约定写进注释（同一个坑第二次踩会很贵）。同时**删掉 world 里的 `<gui><camera name="user_cam">`**，
   机位只留 `gui.config` 一处 —— 两个来源抢同一机位正是"间歇性"的根源（见下）。
2. **禁掉 `CameraTracking` 插件**（`gz_gui_fix.py` 每次启动正则删掉）：world 里有
   `<gui><camera name="user_cam"><view_controller>orbit</…>`，配合该插件后只要 EntityTree 选中
   `robot`，相机就**跟着机器人跑并越缩越近**。实测同一实例 25 s / 50 s / 90 s 三帧分别是
   13 m 全景 → 斜视近景 → 更近，根本没法判断"是否按规划路径走"。删掉后相机固定 13 m 全景。
3. **可视尺寸按 13 m 全景重定**（原来按近景定的，全景下只有几像素）：
   - 轨迹小球 `DOT_R` `0.045 → 0.085`（`gz_viz.py`）
   - 限速圈虚线块 `0.09×0.035×0.012 → 0.14×0.070×0.016`（36 段，`work_area.sdf`）

最后把 `ImageDisplay` 窗口**默认改成 `/overhead/image`**（400×400、锚 3D 视图左上）：
既然它现在真的出图，而 3D 视图在 VMware 上时好时坏，就该让默认视角落在**必然出图**的那路上。
这一张图同时给了四样东西 —— 红色急停盘（r=0.66）、黄色虚线限速圈（r=1.60）、
青/红轨迹点、品红 Nav2 规划点 —— 正好覆盖"是否按规划走 / 是否在障碍旁来回摆"的判断需求。
要看机器人视角就在窗口顶部 topic 下拉切到 `/camera`。

验收图（`v2_crop.png` 放大后）：机器人在粉盘正中、外圈黄虚线完整、青色轨迹点连成一条清晰折线
（走过哪儿一目了然）、品红规划点在上方另一条线上 ⇒ **实际路径与规划路径的偏差可以直接看出来**。

（二）**GUI 3D 视图的"整块空白"也是同一个 bug，不是虚拟显卡**

上一条修完，3D 视图还是空白，但有个被忽略的细节：那块空白是 **#b8cee0 天蓝色**，
和俯视相机朝上时拍到的天空**一个像素不差**。既然能画出天空，GL 显然是好的
（`Accelerated: no` 那条 `glxinfo` 结论把方向带偏了）。

顺着查，`MinimalScene` 的 `<camera_pose>` 用的是**和 camera sensor 同一套约定**
（+X 为前，绕 Y **正**转才把 +X 转到 −Z 朝下）。原来写 `0 0 13 0 -1.5708 0`
= 朝**上**看天，于是整块只剩天蓝色 —— 这正是"还剩底色"这种表现极具误导性的原因。
之前偶尔能渲染的那几次，是因为 world 里还留着 `<gui><camera name="user_cam">`
（`pose 0.001 -0.001 16 0 -1.56 0`，**那一处**是负角朝下），两个来源抢同一个机位，
谁生效看时机 —— 这才是"间歇性"的真正来源。

三处一起改掉：

| 改动 | 内容 |
|---|---|
| `work_area.sdf` | 删掉 `<gui><camera name="user_cam">`，机位**只留 gui.config 一处**（两个来源抢是"间歇性"的根源） |
| `gz_gui_fix.py` | `CAM` 改 `0.0 0.0 13.0 0 **+1.45** 0`（正角 = 朝下；抬 5° 避开正好垂直的奇异位形，顺带能看出障碍高度） |
| `gz_gui_fix.py` | 继续每次启动删 `CameraTracking`，避免相机跟着选中实体跑 |

结果：**3D 视图稳定渲染**，两帧间隔 40 s 都是完整的（房间 + 机器人 + 粉红急停盘 +
黄色虚线限速圈 + 青/红轨迹点 + 品红规划点），且与 `ImageDisplay` 的俯视窗**同时**可用。

> 最终结论：`glxinfo` 的 `Accelerated: no` 确实存在，但它**不是** 3D 视图空白的
> 原因。真正的两个 bug 都是 **pitch 符号**：camera sensor 一处、MinimalScene
> `camera_pose` 一处；两次都因为"画面只剩一种颜色"被误判成渲染崩溃。
> 教训：**只出现纯色时先算朝向，别先怀疑渲染器**；判"相机是否朝对"最便宜的办法是
> 取原始帧数颜色数（1 色 = 指错方向，上千色 = 有几何）。

## 17. 待办（P76 收尾后）

1. **`in_band` 拉回来**：现在稳态落在 1.04 m（目标 1.00），先调 `target_d` 的落脚偏置，
   不要再退回"顶着人走"。
2. `dist < 0.7 m` 的方位估计不可靠（§16.7 遗留 1）——加方位低通 + 迟滞，或按 `dist` 分段增益。
3. `w` 换向 0.77 次/s 的滞回检查（`esc_dir` 左右余量接近时是否仍在翻）。
4. `对正` / `扫描` 的显式仲裁器（优先级 + 最短保持时间）。

---

## 18. §17 四条待办实施与验证（P77，2026-10-26 11:20–12:40）

### 18.1 结论速览

| 待办 | 结论 | 关键证据 |
|---|---|---|
| §17-1 稳态距离 | **完成**，`vis_bias_m` 0.32→**0.357** | d 中位 1.043→**1.008**，in_band 33.4→**42.5%** |
| §17-2 近距离方位 | **完成**（第二版），冻结 + EWMA 0.45 | 居中 ≤10° 95.0→**97.7%**，max 38.0→**22.5°** |
| §17-3 `w` 换向滞回 | **完成**，但"换向次数"本身是错的指标 | 受控场景换向 62→**9**（0.84→**0.12 次/s**），偏航 churn 1777°→**0°** |
| §17-4 对正/扫描仲裁 | **完成**，仲裁器 + ALIGN 时间滞回 + **持续受阻才驻留** | mode 切换 56→**10**，owner 唯一化 |

最终评测版本 **t24_p77f**（`p75_eval.sh 120 t24_p77f rpl`）：

> ⚠️ **本节所有数字都是单次评测**。P78 实测同配置重跑 `in_band` 差 ±6.3 pt
> （见 §18.9），所以下面这张表的**排名不可信**，只有机理类结论成立；
> 三次重复的可靠对比见 §19.2。

| 指标 | t18_p76（§17 前） | **t24_p77f（最终）** |
|---|---|---|
| d 中位 / 均值 | 1.043 / 1.034 | **1.008 / 0.990** |
| **in_band%（0.85~1.15）** | 33.4 | **42.5**（历史最好） |
| near% / far% | 26.4 / 40.2 | 30.3 / **27.2**（历史最好） |
| d std | 0.166 | **0.156** |
| 居中 ≤10° | —（旧 json 无此项） | **97.7** |
| max_deg | — | **22.5** |
| min_d | 0.679 | 0.590 |
| 丢人次数 | 0 | **0** |

### 18.2 §17-1：把稳态距离拉回 1.00 m

**不是比例误差，是固定偏移**。三份评测里 `vis_d - 真值d` 的中位偏差是
−0.356 / −0.358 / −0.357，按距离分档（0.6/0.8/1.0/1.2/1.4 m）始终落在
−0.345 ~ −0.361，而比值从 0.56 一路涨到 0.70 —— 说明测距没有尺度误差，只有
**恒定的近端偏置**（人体的近端边缘像素 + 双目基线假设）。所以只改偏置：

```python
'vis_bias_m': 0.357,   # 原 0.32
```

效果：d 中位 1.043→1.006（t19），in_band 33.4→40.0。**没有再动 `target_d`**，
即没有退回"顶着人走"。

### 18.3 §17-2：`dist < 0.7 m` 的方位估计

近距离时人体占满 front 画幅，`brg` 会在相邻相机之间**跳变**，而控制器又把这种
跳变当成"人换边了"，于是对正与扫描交替翻向。两版尝试：

| 版本 | 做法 | 结果 |
|---|---|---|
| t20_p77b | 给 `brg` 加 60°/s 角速率限幅 | **失败**：in_band 40.0→35.2、min_d 0.634→**0.473**、max_deg 21.6→**38.0**。限幅把**真实**的跨相机切换也拖住了，机器人朝错方向转 1~2 s |
| t21_p77c | `dist < brg_close_m(0.90)` 时：① 关掉"跨相机切换"豁免；② 大跳（>35° 且 Δdist>0.28 且换相机）时**冻结 0.6 s**；③ EWMA 系数 0.55→**0.45** | **成功**：居中 ≤10° **99.3%**、max_deg 24.4、丢人 0 |

结论：近距离方位问题的本质是**别相信那一帧的跳变**，而不是"转得不够快"。

### 18.4 §17-3：`w` 换向次数是个坏指标

受控场景（人绕立柱走一圈后站定，75 s，`PW_REPLAY=/tmp/scn_pillar.jsonl`）四版对比：

| 版本 | w 换向 | 次/s | mode 切换 | 后 40 s 净位移 | 路径长 | **偏航 churn** | 尾段 \|brg\| |
|---|---|---|---|---|---|---|---|---|
| 基线 | 60 | 0.80 | 56 | 0.29 m | 1.27 m | 1662° | 21° |
| +仲裁（符号滞回） | **72** | 0.97 | 60 | 0.13 m | 1.08 m | 1534° | 20° |
| +模式时间滞回 | 66 | 0.89 | 37 | 0.14 m | 1.52 m | 1730° | 21° |
| +急停吸住/释放滞回 | 62 | 0.84 | 49 | 0.12 m | 1.03 m | **1777°** | 23° |

前三版都是"指标动了，问题没解决"：换向次数在 60~72 之间，而**后 40 s 净位移只有
0.1~0.3 m、偏航却累计 1500~1800°**（净转 20~40°）—— 这是标准的**原地打摆**。
所以真正的判据应该是 (净位移, 偏航 churn)，而不是换向次数：

```python
# tmp_p77_an.py 新增
def wander(rows, tail_s=40.0):   # → (净位移, 路径长, churn°, 净转角°)
```

**打摆的机理**（diag 逐帧 + note 实证）：

1. 人在 1.8~2.5 m、偏 2~23°，正前方 0.64 m 是立柱 → `d_head <= stop_d` → 急停；
2. 急停分支把 `w` 强行改成 `1.15 * _esc_dir`（**背离人**的方向）；
3. 下一帧 `d_head` 稍微恢复、急停释放 → ALIGN 立刻把 `w` 掰回**朝向人**；
4. 同一帧的 note 里能同时看到"对正 −5°"和"全向急停 前0.63"，`w = +1.15`
   —— 一个 1.4 s 周期的乒乓。

即：**不是 `esc_dir` 自己在左右翻**（它已有 0.35 m 换边余量 + 0.45 s 保持），
而是"急停要背离、对正要素面"两套意图反号。

### 18.5 §17-4：仲裁器 + "持续受阻才驻留"

#### (a) 转向唯一出口 `_arb_w()`

所有决定转向的模块（avoid / scan / align / track）都必须过这一个函数：

- 优先级 `detour(4) > avoid(3) > scan(2) > align(1) > track(0)`，高优先级**可立即抢占**；
- 同级/降级交接要求双方各保持 `w_owner_hold_s=0.50`；
- owner 内部反向要求持续 `w_hold_s=0.45` 或幅值 ≥ `w_flip_urgent=0.90`；
- **不允许翻转时回 0**（第一版是"沿用旧方向 + 新幅值"，实测反而把换向从 60 抬到 72
  —— 硬顶会过冲，加剧随后的过零）。

同时给 ALIGN 加了**进入/退出时间滞回**（`align_enter_s=0.25`、`align_exit_s=0.35`）：
此前 ALIGN↔TRACK 边界每 1.3 s 抖一次，mode 切换 56 次/75 s；加了之后 56→37。

#### (b) 受阻策略：三种做法 A/B

| 版本 | 策略 | 受控场景（churn / 换向） | 回放评测（in_band / 居中≤10 / min_d / 丢人） |
|---|---|---|---|
| t21_p77c | 不驻留，急停就"哪边空往哪绕" | 1777° / 62 | 37.4 / 99.3 / 0.649 / 0 |
| t22_p77d | **一受阻就驻留**（无限期） | **1°** / **7** | 34.5 / 98.9 / 0.597 / 0 |
| t23_p77e | 驻留 `hold_max_s=2.0` 后**强制定向绕行** | 1407° / 31 | 35.1 / 96.5 / **0.451** / **1** |
| **t24_p77f** | **持续受阻 ≥ `hold_after_s=0.8` 才驻留** | **0°** / **9** | **42.5** / 97.7 / 0.590 / 0 |

- t22 把"1~2 s 就能绕过去"的瞬时阻挡也冻住了 → in_band 掉 2.9 pt；
- t23 更糟：车头掰开 78° 后照样往前顶，min_d 掉到 0.451、丢人 1 次；
- t24 只加"持续性"这一个门槛，同时拿到**最好的 in_band(42.5) 和最好的 churn(0°)**。

#### (c) 顺带修掉的急停乒乓

急停的"吸住/释放"给了两个不同阈值（原来是同一个 `stop_d`，`d_head` 在阈值上下浮动
就每帧吸/放一次）：

```python
'estop_release_k': 1.25,   # 释放需前方净空 > 0.825 m
'estop_max_s': 2.50,       # 最长吸住, 兜底防窄角里永远等不到释放条件
```

### 18.6 最终参数（`vla_demo/vision_config.json` + `DEFAULT_CFG` 同步）

```json
"vis_bias_m": 0.357,      "brg_close_m": 0.9,      "brg_af_close": 0.45,
"w_hold_s": 0.45,         "w_owner_hold_s": 0.5,   "w_flip_urgent": 0.9,
"w_arb_on": 1,            "align_enter_s": 0.25,   "align_exit_s": 0.35,
"estop_release_k": 1.25,  "estop_max_s": 2.5,
"hold_blocked_m": 2.5,    "hold_after_s": 0.8,     "hold_max_s": 2.0,
"detour_s": 1.3,          "detour_dir_hyst": 0.35, "detour_w": 1.05,
"detour_on": 0
```

`detour_on=0` 是**有意保留的 A/B 开关**：定向绕行窗口的代码路径完整可用
（窗口内 owner=detour，优先级最高，ALIGN/TRACK 都抢不走），但实测全面劣于
"持续受阻才驻留"，所以默认关闭。

### 18.7 变更文件

| 文件 | 改动 |
|---|---|
| `vla_ws/src/vla_follow/vla_follow/follower.py` | `_arb_w()`；`_arb_w` 接入 avoid/scan/align/track 四个出口；ALIGN 时间滞回；急停吸住/释放；持续受阻驻留 + 定向绕行窗口；近距方位冻结；`vis_bias_m` |
| `vla_demo/vision_config.json` | 新增 16 个参数（见 18.6） |
| `tmp_p77_an.py` | 新增 `wander()`：尾 40 s 净位移 / 路径长 / 偏航 churn / 净转角 |
| `deploy/follower.py` | 同步（旧版留 `deploy/_follower_p76_old.py.bak`） |

### 18.8 遗留

1. **`min_d` 0.590**：低于 `lidar_stop_m=0.66`，是驻留解除瞬间的一帧冲量（`_safety_gate`
   在 10 Hz 上，0.16 m/s × 0.1 s ≈ 0.016 m 不够解释）。下一步应在 `_publish_smooth`
   的斜坡里给"驻留→前进"的恢复加 0.3 s 软启动。
2. **居中 ≤10° 97.7% vs 99.3%**（t21）：多出来的 1.6 pt 全部集中在"驻留刚解除"的
   0.3~0.5 s 内（`p90` 二者都是 6.1~6.3°，`median` 2.3~2.5°，差别只在个别事件）。
   与上面第 1 条是同一个软启动问题。
3. `w` 换向次数在正常跟随段仍有 ~0.4 次/s，但已不再对应可见的摆动（churn 在受控
   场景为 0°）。若后续还要压，方向是"把 TRACK 的 `|brg|>3°` 死区放大到与
   `aim_band_deg` 一致"，而不是继续加滞回。

### 18.9 ⚠️ 噪声底：单人单次的排序不可信（P78 追加）

P78 做 A/B 时发现：**同一份代码、同一份回放、同一条命令跑两遍**，结果差这么多：

| 同一配置 | in_band | near | far | 居中≤10° | max_deg | min_d | lost |
|---|---|---|---|---|---|---|---|
| t24_p77f | 42.5 | 30.3 | 27.2 | 97.7 | 22.5 | 0.590 | 0 |
| t29_p78e（重跑） | 36.2 | 28.8 | 35.0 | 99.4 | 16.1 | 0.681 | **1** |
| **差** | **−6.3** | −1.5 | **+7.8** | +1.7 | −6.4 | **+0.091** | — |

人已经是开环回放（= 时间的纯函数），噪声全部来自**被控对象自己的反馈**：
机器人决策 → 轨迹 → 感知 → 决策，微小调度差异被放大成完全不同的轨迹（混沌）。

**因此 §18.5 的表格只能当"机理证据"读，不能当"排名"读**：

| 结论 | 是否成立 | 依据 |
|---|---|---|
| 打摆真因是"急停背离 ↔ 对正朝人"反号 | ✅ 成立 | 受控场景 diag 逐帧 note + churn 1777°→0°，量级差 3 个数量级，不是噪声 |
| `hold_after_s` 门槛（t24 结构）| ✅ 成立 | 同上；且它只影响"受阻≥0.8s 才驻留"这一条硬逻辑 |
| t24 in_band 42.5 是历史最好 | ❌ **撤回** | 同配置重跑只有 36.2，42.5 是运气好的一次 |
| t22/t23 的 in_band 差 2~3pt 说明策略更差 | ❌ **撤回** | 全在 ±6pt 噪声内 |
| 定向绕行（t23）会丢人、min_d 0.451 | ⚠️ 存疑 | min_d 噪声 ±0.09，但 0.451 vs 0.681 仍偏低；"车头掰开还往前顶"的机理仍成立 |

**新口径**：任何 < 7 pt 的 `in_band` 差异、< 0.1 m 的 `min_d` 差异，单次评测**不足以判定**。
必须 ≥3 次重复取中位（`tmp_eval_rep.sh`），要分辨 2~3 pt 的差异得 6~10 次。

## 19. 距离过近的处理与评测可靠性（P78，2026-10-26 12:45–13:10）

### 19.1 两个候选修法，一个被证伪

§18.8 的遗留 1/2 说"`min_d` 偏低、居中度略降，都集中在驻留刚解除的 0.3~0.5 s"。
P78 试了两个修法：

#### (a) 前进软启动 —— **证伪**

`resume_soft_s`：受阻解除后的 0.3 s 内把正向 `v_t` 按线性从 0 升到全速
（只压正向，后退不受影响）。预期是把解除瞬间的冲量削掉。

实测 **t25_p78a 全面崩坏**：in_band 42.5→23.1、near 30.3→**54.9**、居中 97.7→**73.3**、
min_d 0.590→**0.410**、丢人 +1。

失败链条（recs 逐帧）：

```
t=73.2  d=0.99  brg=-3°  cmd_v=+0.21   TRACK
t=73.6  d=0.81  brg=11°  cmd_v=+0.22   TRACK
t=74.0  d=0.68  brg=11°  cmd_v= 0.00   ALIGN ← 越界(>10°)+过近, 只转不进
t=74.4  d=0.58  brg= --   cmd_v= 0.00   检测丢失
t=75.2  d=0.41  ...
t=75.6  d=0.41  SCAN   ← 之后 45s 都在找
```

软启动让恢复段慢了几拍，人继续朝机器人走；等 `dist` 掉到 `target_d-0.15` 以下时，
ALIGN 分支按设计"只转不进"（v=0），而人还在走 → 距离被压到 0.41 m → **front 相机
被人体占满画幅 → 检测丢失 → 进 SCAN**。软启动把"停车让距"这个缺口放大了。

结论：**`resume_soft_s` 默认 0**（代码保留，只在需要时开）。

#### (b) 过近时"停"改成"比例让距" —— 采用

真正的问题是 (a) 暴露出来的那个缺口：**过近时只会停，不会退**。人朝机器人走过来时，
"停"等于把距离的决定权完全交给对方。改成按越线深度比例后退：

```python
if dist < self.target_d - 0.15:          # 0.85m 以内
    v_t = 0.0
    if close_back_on and dist < self.target_d - close_back_m:   # 0.85m
        if 后方净空 > lidar_stop_m + 0.30:
            gap = (target_d - 0.15) - dist
            v_t = -min(0.28, close_back_v + close_back_k * gap)  # 0.10 + 0.8*gap
```

刚越线只退 0.10 m/s（不扰动距离环），越近退得越快（0.6 m 时 0.30→限 0.28）。
固定 −0.22 的版本（t26）在 0.84 m 也一路退，把 `far` 抬了 3.2 pt；比例版（t28）没有。

按 §18.9 的口径，t26/t27/t28 与 t24 的差异**都在噪声内**，所以选择的依据不是数字而是机理：
"人贴到 0.6 m 内"是明确要避免的（会导致丢目标、且违背"保持设定距离"），
代价在噪声内 ⇒ **`close_back_on=1`**。

### 19.2 三次重复的净效果（同一口径，`tmp_eval_rep.sh 3`）

“旧参数”= 用配置回退到 §17 之前的行为（`vis_bias_m` 0.32、关掉仲裁/滞回/驻留/让距）。

| 指标（3 次中位） | 旧参数 | **P78 最终** | 变化 |
|---|---|---|---|
| in_band% | 35.4（35.2~37.8） | **39.1**（35.4~41.4） | +3.7（**在噪声内，不算显著**） |
| far% | 37.2（36.9~38.5） | **31.2**（27.9~34.0） | **−6.0** |
| near% | 27.4 | 30.6 | +3.2 |
| d 中位 | 1.04 | **1.02** | −0.02 |
| 居中 ≤10° | 97.0（96.2~98.9） | **98.1**（97.0~98.4） | +1.1 |
| **max_deg** | **35.2**（21.9~38.9） | **20.6**（19.1~22.3） | **−14.6°，且分布收紧** |
| **min_d** | **0.46**（0.43~0.70） | **0.67**（0.64~0.68） | **+0.21，且不再下探 0.43** |
| 丢失事件 | 1/1/0（均 < 0.3 s） | 0/2/0（均 < 0.3 s） | 无实质差异 |

**可以站住的结论**（差距远大于噪声）：

1. **`max_deg` 35.2° → 20.6°**，且三次分布从 21.9~38.9 收到 19.1~22.3 —— 这就是
   “障碍旁来回摆动”被消掉的直接证据（与受控场景 churn 1777°→0° 一致）。
2. **`min_d` 0.46 → 0.67**：旧参数会把人顶到 0.43 m（人体占满 front 画幅 → 必然丢目标），
   新参数三次都没低于 0.63。这是 §17-1 距离偏差校准 + 驻留 + 让距三者叠加的结果。
3. **`far` 37.2% → 31.2%**：不再长时间吊在 1.2 m 以外。

**不能站的**：`in_band` 的 +3.7 pt。三次极差本身就有 6 pt，样本量不足以判定。

### 19.3 参数

```json
"close_back_on": 1,   "close_back_m": 0.15,
"close_back_v": 0.10, "close_back_k": 0.80,
"resume_soft_s": 0.0
```

### 19.4 变更文件

| 文件 | 改动 |
|---|---|
| `follower.py` | ALIGN 过近分支加比例让距；`resume_soft_s` 软启动（默认 0）；`close_back_*` 参数 |
| `vla_demo/vision_config.json` | 同名 4 个参数 |
| `tmp_eval_rep.sh` | **新增**：同一配置重复 N 次评测并输出中位/极值（解决 §18.9 的噪声问题） |

### 19.5 遗留

1. **评测本身要改口径**：`p75_eval.sh` 单次结果不足以判定 A/B，后续方案对比一律走
   `tmp_eval_rep.sh N≥3`；若要做正经的参数搜索，应该固定随机种子并把物理步长与
   控制周期解耦（现在噪声主要来自调度抖动）。
2. 软启动的思路本身未必错，错在"只压加速度、不修停车缺口"。若再试，应当与
   "过近必须退"同时生效，并单独验证不丢目标。


---


## 20. 跟随距离区间 0.5~1.5 m + 接近减速（P79，2026-10-26 13:00–14:00）

### 20.1 需求与落地

用户口径两条：

1. 跟随距离区间改为 **0.5~1.5 m**；
2. 接近过程中 **人距 < 2 m 必须开始减速**，避免撞上和频繁后退。

只改一个 `track_dist_tol_m` 是不够的 —— 带宽从 ±0.1 放到 ±0.5 之后，控制器里
所有"按窄带写死"的门限都会在**合法区间内**触发。一并改了五处：

| 位置 | 原值（±0.1 窄带） | 新值（0.5~1.5） | 不改会怎样 |
|---|---|---|---|
| `track_dist_tol_m` | 0.10 | **0.50** | — |
| `guard_d` | `max(0.55, ...)` | `max(slow_floor+0.05, ...)` = 0.55 | 与新下限脱钩 |
| TRACK 硬下界 | `dist<0.82 → v_cap=0` | `dist<slow_floor+0.02=0.52` | 0.82 是新区间**中部**，在那儿剁停会让人走进 0.5 内丢目标 |
| ALIGN 过近门限 | `target_d-0.15=0.85` | `target_d-close_back_m=0.50` | 0.55~0.85 全是合法区，仍按越界处理 → 不停后退 |
| `close_back_m` | 0.15 | **0.50** | 同上 |

另外 PID 的误差在带边缘从 0.1 变成 0.5，同一 `kp=1.9` 会把反馈打到 ±0.45 的
饱和平顶（距离刚进 0.55 m 就一脚倒车）。所以加 `inband_fb=0.4` 软死区：
带内增益 ×0.4，出带连续接回原增益（折点处值相等，无指令跳变）。

```python
if abs(err_d) <= tol:
    err_d *= ins                                          # 带内: 0.15 -> 0.06
else:
    err_d = copysign(abs(err_d) - tol * (1 - ins), err_d)  # 出带: 斜率仍为 1
```

### 20.2 接近减速曲线

原来只有 `v_lat = v_ff + (dist-guard_d)/0.8` 一条隐式限速，**2.0 m 处它约等于 1.8,
完全不起作用**，等于"看见人还在全速冲"，冲到 0.8 m 再由硬下界剁成 0，然后
距离环反向拉回 —— 这正是"撞上 + 频繁后退"的组合。

新增单调包络（TRACK 与 ALIGN 两个前进出口共用 `_slow_k(d)`）：

```
k = clamp((d - slow_floor) / (slow_zone - slow_floor)) ** slow_exp
slow_zone = 2.0, slow_floor = 0.5, slow_exp = 0.45
```

| 人距 m | 3.0 | 2.0 | 1.8 | 1.5 | 1.3 | 1.1 | 1.0 | 0.8 | 0.7 | 0.6 | 0.55 | 0.52 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 系数 | 1.00 | 1.00 | 0.95 | 0.83 | 0.76 | 0.68 | 0.62 | 0.50 | 0.42 | 0.31 | 0.22 | 0.16 |
| v 上限 | 0.72 | 0.72 | 0.68 | 0.60 | 0.55 | 0.49 | 0.45 | 0.36 | 0.30 | 0.22 | 0.16 | 0.11 |

`slow_exp` 取 0.45 是实测调出来的：第一版取 0.6，1.3~1.5 m 的速度上限被压到
0.49~0.56，人稍快就跟不上 → 距离被拉到 1.9 m → 超带（far 28%）且更容易丢目标。

### 20.3 "频繁后退"的真身：前馈在带中心反向

t24（旧版）在 **人距 1.0~1.1 m 的带中心**上，仍有 **41.7% 的帧在下发后退、
40 次/100 s**。根因不是距离环，而是 P40 的人速前馈：人朝机器人走来时
`_pvvr < 0` → `v_ff` 直接变 −0.30，加 PID 后 `v_t` 变负。

**人的来向不该由机器人倒车抵消。** 加后退闸门：

```python
_bg = slow_floor + 0.20          # 0.70 m
if v_t < 0.0 and dist > _bg:
    v_t = 0.0                    # 只收油门停住, 不倒车
```

0.55 m 以下仍由 `guard_d` / 8 段防撞人发后退，该退的时候照退。

三个候选口径各跑 3 次重复（`tmp_eval_rep.sh 3`）：

| 口径 | back 帧占比 | 后退次数/100s | d 中位 | min_d | lost |
|---|---|---|---|---|---|
| 旧版（无闸门） | 41.7% | 40.8 | 1.008 | 0.590 | 0~1 |
| A. 0.70 m 硬闸门 | **1.4%** | **2.8** | 0.94 | 0.41 | 3 |
| C. 0.80 m 闸门 + "距离变化率<0" 例外 | 29.4% | 39.8 | 1.03 | 0.41 | 0~2 |

C 多出来的 28 个百分点后退**没有换来任何 `min_d` 改善**（两者都是 0.41）：人走
走停停让 `_pvvr` 频繁变负，判据分不清"真的逼近"和"距离抖动"。按"数字差在噪声
内时选机理正确的那个"，取 A。

### 20.4 三次重复评测（`tmp_eval_rep.sh 3 p79c 120`，回放口径）

`in_band` 口径已改为**从配置读**（`track_dist_m ± tol` = 0.5~1.5），不再硬编码
0.9~1.1：

| 指标 | 中位 | 最小 | 最大 |
|---|---|---|---|
| `in_band`% | **98.6** | 96.9 | 99.2 |
| `near`% | 1.4 | 0.8 | 3.1 |
| `far`% | **0.0** | 0.0 | 0.0 |
| d 中位 m | 0.94 | 0.93 | 0.94 |
| 居中 ≤10°% | 96.4 | 90.6 | 97.1 |
| back 帧占比 % | **1.4** | 1.1 | 4.2 |
| 后退次数/100 s | **2.8** | 2.8 | 4.7 |

两点必须说清，否则会误判成退化：

1. **`min_d=0.41` 不在稳态**。四份录像（含 t24）**开局都要 ~10 s 才首次检出**
   （相机/检测器预热），这段由 `dist=真值`、`det=0` 组成，不参与 `in_band`
   统计但会进 `min_all`。t24 同样有，只是它的 lost 事件边界刚好把这段算掉了。
2. **`max_deg` 32~39° 不是摆动**。逐帧看全发生在"人从 left/right 相机切进
   front 的过渡首帧"（brg 102° → 76° → 38° → 11° → 0°，0.7 s 内收敛）。
   稳态 `|brg|` 中位 **1.9~2.3°**，比 t24 的 2.5° 还小。

### 20.5 demo 语音把目标距离钉在 2.0 m

排查 `td=2.0` 时发现：`voice_trigger.py` 默认 wav 是 `follow_2m.wav`
（"请跟随我，保持两米的距离"），每次 demo 启动都把 `target_d` 设成 2.0 m ——
与 0.5~1.5 的设定直接冲突，且 2 m 时 `err_d=+0.8` 让 PID 长期饱和前进。

两处修：① `_cmd_cb` 把外部请求的 `distance` **钳位到 track_dist_m ± tol**，
越界只告警不生效；② 默认 wav 换成 `follow_me.wav`（不带距离），由配置给 1.0 m。

### 20.6 参数

```json
"track_dist_m": 1.0,  "track_dist_tol_m": 0.5,
"slow_zone_m": 2.0,   "slow_floor_m": 0.5,   "slow_exp": 0.45,
"inband_fb": 0.4,     "close_back_m": 0.5
```

### 20.7 变更文件

| 文件 | 改动 |
|---|---|
| `follower.py` | `_slow_k()` 减速包络；TRACK/ALIGN 接包络；软死区；后退闸门；`guard_d`/硬下界/ALIGN 门限跟区间下限走；`_cmd_cb` 距离钳位 |
| `voice_trigger.py` | 默认 wav → `follow_me.wav`，日志同步 |
| `vla_demo/vision_config.json` | 5 个新参数 + `tol`/`close_back_m` 修改 + `person_keep_m`/`person_slow_m` 对齐（这两项代码已不使用） |
| `tmp_random_eval.py` | 距离口径从配置读（`--band` 可覆盖）；新增 `back` 段（占比/事件数/每百秒/平均时长） |
| `tmp_eval_rep.sh` | 汇总表加 `bk`/`bn`/`be` 三列 |
| `tmp_gz_snap.py` | 修竞态：`done` 改为写盘完成后才置位（原来主线程先 `os._exit`，图不落盘） |

### 20.8 遗留

1. ~~**`min_all` 的 0.41 里有一半来自开局预热**~~ —— **此结论已被 §21.1 推翻**。
   把最小值对应的时刻逐一定位，三次分别在 t=88.8 / 29.3 / 90.4 s，且都紧邻一次
   检测丢失；人裙摆 0.26 + 车体 0.18 = 0.44 m 即物理接触，所以 0.41 是**真撞上**，
   不是统计混入的预热帧。开局预热（~10 s 才首次检出）确实存在，但它不是最小值
   的来源。把统计窗口推后只会把问题藏起来。
2. **`d` 中位 0.94 而非 1.00**：后退闸门禁止在 0.70 m 外倒车，人走动时机器人
   只能停，距离自然偏近 6 cm。完全在 0.5~1.5 内，暂不动。
3. **x11grab 抓屏失效**（`Error opening input: Invalid argument`，场景本身正常）。
   现用 gz 话题抓图代替；若以后要 GUI 截图，需把 `tmp_xwd2png.py` 的头部解析
   补完（实测 `byte_order`/`bits_per_pixel` 偏移与规范不符）。
4. `slow_exp` / `slow_zone_m` 只在 0.5~1.5 区间下调过，将来若再改区间需重调。


---


## 21. 撞人归因："盲走"才是根因（P80，2026-10-26 14:10–15:10）

### 21.1 先把上一节的错误结论撤掉

§20.8 写的是"`min_all` 的 0.41 里有一半来自开局预热"。把三次评测里最小值对应的
时刻逐一定位后，这个说法站不住：

| 录像 | 全段 min | 时刻 | t>12 min | 时刻 | det&非丢失 min |
|---|---|---|---|---|---|
| p79c_1 | 0.410 | 2.6 s | 0.410 | **88.8 s** | 0.429 |
| p79c_2 | 0.410 | 10.4 s | 0.416 | **29.3 s** | 0.449 |
| p79c_3 | 0.410 | 7.2 s | 0.410 | **90.4 s** | 0.613 |

三处的共同点：**紧邻一次检测丢失**，且 `det=0` 的帧占多数（70~92%）。把
`work_area.sdf` 里的几何翻出来算一下就知道这不是"擦边"：

```
人 skirt 碰撞圆柱 radius = 0.26
车体外接半径             = 0.18
中心距 0.44 m 即物理接触  →  min_d 0.41 是 penetrated
```

所以它不是统计口径问题，是真撞上。把统计窗口推后只会把问题藏起来 —— 这也是
这一节没有按"等首次检出再计时"去做的原因。

### 21.2 归因手段：把决策文字挂进评测

之前的评测只记"距离/角度/指令"这类**结果量**，无法回答"为什么 0.65 m 还在给
`v=+0.23`"，只能事后猜机理。这一轮给 `tmp_random_eval.py` 加了 `/follower/diag`
订阅，每条 `rec` 多记 `note / mode / td / sl / dh / dl / dr / dw`；`follower.py`
的 `_dbg` 再加 `vt / pv / vr`（闸门后指令、人速估计、本机前向速度）。

有了它，一次评测就能把失效链条直接读出来（t32，76.0~76.4 s）：

```
t=75.62 d=0.69 vis= 0.36 det=1 cam=front mode=ALIGN v=+0.07 | 对正+8°(边转边走)
t=75.82 d=0.62 vis=-1.00 det=0 cam=-     mode=ALIGN v=+0.22 | 对正+8°(边转边走)
t=76.02 d=0.54 vis=-1.00 det=0 cam=-     mode=ALIGN v=+0.22 | 对正+8°(边转边走)
t=76.22 d=0.45 vis=-1.00 det=0 cam=-     mode=ALIGN v=+0.22 | 对正+8°(边转边走)
t=76.42 d=0.41 vis=-1.00 det=0 cam=-     mode=ALIGN v=+0.07 | 对正+8°(边转边走)
```

**视觉已经连续 4 帧没检出，ALIGN 的"边转边走"还在按陈旧的 last-known 距离
判"没到目标距离"，以 0.22 m/s 前推 1.0 s，把 0.62 m 一路顶到 0.41 m。**

同期的深度相机 `dh` 报 **99.00**（无回波）—— 0.4 m 以内是近距盲区。也就是说
这个距离上**没有任何传感器能兜底**。

### 21.3 四处修法

**(a) 盲走禁令**（`blind_v_s=0.30`，`_publish_smooth` 里）

```
_lvd = now - self._vis_mono          # 距上次"检出"的时长
if v_t > 0.0 and _lvd > blind_v_s:
    v_t = 0.0                        # 只压正向, 后退/转/停都允许
```

放在梯形限幅**之前**，所以是"刹车"而不是"瞬断"。0.30 s 在 10 Hz 检测下等于
"连续 3 帧没检出"，单帧漏检不受影响。

**(b) 主动让距**（`back_hold_*`，TRACK 与 ALIGN 两个前进出口都接）

P79 的后退闸门是"0.70 m 以外禁止倒车"。问题是**检测器在真值 0.70 m 就已经
看不见人**（实测 `vis` 原始值 0.39 → `detected=false`，人体占满画幅）。于是
"停在 0.77 m 等人自己走完"这个策略是自相矛盾的：停 0.3 s 后就瞎，然后人走进
接触距离。0.70 m 这条线恰好卡在"已经瞎了"的边界上。

改成 0.85 m 以内**主动让距**，速度跟着人的逼近速度走：

```
v_back = min(vmax, max(vmin, gain*max(0, -pv) + k*(0.85 - dist)))   # 上限 0.45
```

`gain=1.0` 意味着"人以多快逼近就退多快"（距离保持不变），`k=0.7` 那一项负责
把距离缓慢拉回 0.85。人停下 → `pv≈0` → 只剩 `k` 项，退到 0.85 就停，**不会**
像 P79-C 那样"距离一抖就倒车"（那版 back 29.4% 且 min_d 无改善）。

**(c) ALIGN 不再无脑"边转边走"**

原来只要 `dist >= target_d - close_back_m(=0.50)` 就以 0.22 m/s 前推。现在
`dist < 0.85` 走同一套让距/原地转。这是链条里最直接的一环。

**(d) ALIGN 在"已经比目标近"时只转不走**（第二次修正，见 §21.4 的回归）

只做 (a)(b)(c) 之后跑了一次确认（t35），接触又回来了（86 帧、`min_d` 0.41）。
逐帧对比 p80c 与 t35 才发现差别不在阈值，而在**阈值卡在噪声带上**：

```
t35   76.21 d=0.90 vis=0.56 → dist≈0.92  v=+0.22  边转边走   (≥0.85, 不触发)
      76.46 d=0.78 vis=0.50 → dist≈0.86  v=+0.28  边转边走
      76.61 d=0.71 vis=0.40 → dist≈0.76  v=+0.28  ← 估计滞后, 还没触发
      76.66 d=0.68 vis=0.40 → dist≈0.76  v=+0.28  ← 这一帧才切让距
p80c  26.01 d=0.78 vis=0.46 → dist≈0.82  v=-0.06  后退让距     (<0.85, 触发)
```

视觉距离估计在相邻两帧之间会跳 0.10~0.15 m（1° 方位量化 + 检测框抖动），所以
"0.85 m 才让距"必然有 1~2 帧的滞后，那一两帧足够把 0.90 推到 0.70。

根子上的矛盾是：**ALIGN 的职责是转向，不是挪位**。既然 `dist < target_d`（已经比
目标近），距离环本身就想后退，ALIGN 却还在往前挪 —— 两个模块意图相反。所以：

```python
elif dist < self.target_d:       # 已经够近 → 纯旋转对正
    v_t = 0.0
else:                            # 还远 → 边转边走(带减速包络)
    v_t = 0.85 * approach_speed * slow_k(dist)
```

这一条不依赖阈值精度，所以是**确定性**的：p80e 三次全部 0 接触。

### 21.4 结果（`tmp_eval_rep.sh 3`，每版 3 次 120 s 回放）

先把"只做 a/b/c"那次确认跑的回归也列出来，因为它决定了要不要做 (d)：

| 版本 | 接触帧 | `min_d` | `in_band`% | 居中≤10°% | `max_deg` | 丢人 |
|---|---|---|---|---|---|---|
| P79（p79c，改前） | 101 / 44 / 49 | 0.41 | 98.6 | 96.4 | 32~39 | 0~3 |
| P80 a/b/c（p80c） | 0 / 0 / 0 | 0.61~0.70 | 100.0 | 95.9~98.1 | 17.7~36.4 | 0 |
| P80 确认跑（t35，**回归**） | **86** | **0.41** | 98.0 | 95.1 | 42.7 | 2 |
| **P80 a/b/c/d（p80e）** | **0 / 0 / 0** | **0.62~0.73** | **100.0** | **98.7~99.3** | **14.4~17.2** | **0** |

p80e 三次重复的完整口径：

| 指标 | 中位 | 最小 | 最大 |
|---|---|---|---|
| `in_band`% | 100.0 | 100.0 | 100.0 |
| `near`% / `far`% | 0.0 / 0.0 | 0.0 | 0.0 |
| `d` 中位 m | 0.968 | 0.949 | 0.979 |
| 接触帧（d<0.44） | **0** | 0 | 0 |
| `min_all` m | 0.70 | 0.62 | 0.73 |
| 居中 ≤10°% | **98.9** | 98.7 | 99.3 |
| `max_deg` | **16.1** | 14.4 | 17.2 |
| 后退帧占比 % | 11.2 | 9.2 | 15.6 |
| 后退次数/100 s | 13.0 | 11.1 | 21.3 |
| 丢人次数 | **0** | 0 | 0 |

三点解读：

1. **t35 的回归是关键证据，不是失败。** 它证明 (a)(b)(c) 的"0 接触"有运气成分：
   阈值 0.85 与视觉噪声带同宽，触发与否取决于那两帧的估计值。加上 (d) 之后
   不再依赖阈值精度 ⇒ 3 次全 0。**如果只跑那 3 次 p80c 就收工，这一条会留下**。
2. **接触 101→0 是分类级变化**。前面测过同配置重跑的噪声底是 `in_band` ±6.3 pt、
   `min_d` ±0.09；这里是"有/无"的区别（p80e 3 次 + p80c 3 次，共 6 次全 0；
   而 P79 3 次、t35 1 次全部有）。
3. **后退 1.4%→11.2% 是让距换来的，不是抖动。** 后退事件 11~21 次/100 s、
   单次平均 0.8 s，全部集中在人走近到 0.85 m 以内的那几次；稳态（人距 ≈0.95 m）
   依然是 0 后退。用户要避免的"频繁后退"指带中心来回抽动，那一项仍是 0。
   `d` 中位从 0.94 抬到 0.968，也说明让距起作用了而不是白退。

### 21.5 参数

```json
"blind_v_s": 0.30,
"back_hold_m": 0.85,  "back_hold_gain": 1.00,  "back_hold_k": 0.70,
"back_hold_vmax": 0.45, "back_hold_vmin": 0.06, "back_hold_on": 1
```

### 21.6 变更文件

| 文件 | 改动 |
|---|---|
| `follower.py` | `_publish_smooth` 加盲走禁令；TRACK/ALIGN 两个前进出口加主动让距；`_dbg` 增 `vt/pv/vr`；diag 增 `bl`（近 1 s 盲走触发次数） |
| `tmp_random_eval.py` | 订阅 `/follower/diag` 并写入每条 rec；新增 `contact` 段（阈值 0.44 m，按事件聚类，附 `det` 计数与当段 note） |
| `tmp_eval_rep.sh` | 汇总表加 `contact/ctev/ctdmin` 三列 |
| `vla_demo/vision_config.json` | 7 个新参数 |

### 21.7 遗留

1. **0.44 m 的接触阈值是算出来的，不是量出来的**（0.26+0.18）。真值位姿来自
   `dynamic_pose/info`，两个模型都是圆柱/盒体，中心距即最短距离，所以够用；
   若以后换成非圆形底盘，这个数要重算。
2. **`max_deg` 在 p80c 里还有 36.4° 的离群点**（p80e 已降到 14.4~17.2），出现在
   "人贴着 0.7 m 从侧面走过"的那 0.4 s：机器人在后退让距、同时要转 36°，两者
   争夺同一份轮速。稳态 `|brg|` 中位只有 2.5~2.7°。
3. **`back_hold_*` 的 `gain=1.0` 是"距离保持不变"的意思**，不是"拉开距离"。
   若现场希望人靠近时机器人主动退到 1.0 m 以上，把 `back_hold_k` 从 0.7 提到
   1.2 即可（会让 `d` 中位继续上抬，`near` 仍为 0，但后退更频繁）。
4. **深度相机 0.4 m 内无回波**（`dh=99`）是硬约束。近距保护只能靠"不盲走 +
   主动让距"，不能指望传感器。将来若要贴身跟随 0.3 m，需加超声/红外近距传感器。


---
## 22. 障碍附近丢目标（P81，2026-10-26 15:40–18:00）

### 22.1 现象

现场长跑（40 min+）里 `FOLLOWER ✗ 目标丢失` 反复出现，且**几乎都发生在人经过
障碍物的时候**；`VISION 四相机视野内均无人` 累计 267 次。但 120 s 随机行走评测
（p80e）里 `lost.n=0`、`vis_pct=97.8` —— 标准评测抓不到这个问题，所以先做
**受控复现**再谈修。

复现装置（`tmp_orbit_test.py`）：人绕 ob1 立柱（3.5,2.0）以 R=1.0 m、0.35 m/s
匀速转圈，机器人从 1 m 外跟随，每圈有约 1/3 行程被柱子挡住。同时记录
`/follower/status` 的检测状态 + 真值位姿 + **视线几何**（机器人→人的连线是否
穿过障碍 OBB）。

### 22.2 根因（四条，全部有实测数据）

#### 根因一（主因）：人的外观先验与仿真材质**根本不匹配**

`person_vision` 的先验写的是"深色圆柱（V<70 且 S<100）"。实测：

| 物体 | 渲染后的 V | 与阈值 70 的关系 |
|---|---|---|
| 人的裙摆（材质 0.25/0.25/0.28） | **75**（S=27） | **高于阈值 → 判不出来** |
| 地板（0.42/0.5/0.42） | 95 | 只差 20 个灰阶 |
| 墙（0.55/0.55/0.6） | 219 | 远离 |

阈值 70 卡在"裙摆 75"和"地板 95"中间，**往下只有 5 个灰阶余量**。于是掩膜只能
切到裙摆最暗的两条轮廓边（实测 7~9 px 宽、面积 269~374 px、**宽高比 0.15~0.19**），
而 `ASPECT_MIN=0.2` 又把这种细条判死 → 整帧"未检出"。换个观察角度（圆柱受光面
朝向不同）就时好时坏 —— 这正是"跟到障碍附近就丢"的手感来源。

证据：`cap_pillar/` 抓帧离线复现检测器（`tmp_det_trace.py`）—— back 相机 1.12 m
处人清晰可见，但 4 台相机**暗色连通块总数为 0/0/0/2**，那 2 个也被宽高比判死。

#### 根因二：`AREA_MAX=22000` 与需求带下界（0.5 m）冲突

裙摆在 320x240 上的面积 ≈ `3718/d²`（d = 相机到裙摆表面）。22000 对应
d=0.41 m，即**人中心 0.67 m**。需求是"保持 0.5~1.5 m"，也就是**合法带的下
半段（0.5~0.67 m）会被面积上限判死**。

#### 根因三：几何一致性检查把"被障碍切掉一半的人"判成非人

原判据：由固定可见宽度 0.36 m 反推 `d_geo`，要求实测距离在
`[0.38·d_geo, 1.62·d_geo]` 内。人绕到柱子/箱子后面只剩一条残片时，可见宽度
掉到一半 → `d_geo` 翻倍 → 距离跌破下界 → 判"不是人"。**贴着障碍就丢**的直接
原因之一。

#### 根因四（放大器）：相机只有 0.25 m 高，而裙子只有 0.06~0.50 m 高

人身上**唯一**可辨的特征（深色裙摆）在离地 0.06~0.50 m；相机装在 0.25 m；
场景里 7 个障碍有 6 个高度 ≥ 0.5 m。于是任何 ≥ 0.5 m 的障碍在 1~2 m 外就能
把这条特征完全遮住 —— 即使人的躯干/头还在视野里（躯干是亮蓝、头是肤色，
都不满足"暗色"先验）。

#### 附带：`HFOV` 常数写错

`person_vision.HFOV = 1.7453`（100°），而 SDF 里 4 台 rgbd 的
`horizontal_fov = 1.8`（103°）→ 方位角整体偏 3%，视野边缘差 1.6°。

### 22.3 修法（感知）

| # | 位置 | 改动 |
|---|---|---|
| 1 | `work_area.sdf` | 裙摆材质 `0.25/0.25/0.28` → **`0.07/0.07/0.08`**（渲染 V≈35~45，与地板 95 拉开 50 灰阶，先验才真正成立） |
| 2 | `person_vision.py` | `AREA_MAX` 22000 → **60000**（覆盖到中心 0.43 m） |
| 3 | `person_vision.py` | `ASPECT_MIN` 0.2 → **0.14**（容忍竖直残片） |
| 4 | `person_vision.py` | 几何一致性改成**由深度反推可见宽度**并限制在 `[0.09, 0.60] m`：残片天然容忍，暗色墙面（反推 4~5 m 宽）照样剔除 |
| 5 | `person_vision.py` | `HFOV` 1.7453 → **1.8** |
| 6 | `work_area.sdf` | 4 台 rgbd `update_rate` 5 → **10**（见 22.6） |

`DEPTH_MIN=0.28` **故意不动**：机器人自身车体/脚轮在 0.10~0.25 m 处也是深色，
0.28 是"排除自遮挡"和"跟到 0.54 m"的分界线。

### 22.4 受控复现结果（绕柱 70 s，人贴柱 pd≈0.19~0.5 m）

| 版本 | 检出率 | 丢失事件 | 距离中位 / min / max | \|brg\| 中位 / p90 |
|---|---|---|---|---|
| o1 基线（改前） | **30.3 %** | 10 次 / 15.7 s | 1.56 / – / 2.00 | – |
| o2 只改感知 | 73.7 % | 10 次 / 15.7 s | 1.56 / – / 2.00 | – |
| o6 感知（5 Hz 相机） | **100 %** | **0 次** | 1.08 / 0.57 / 1.53 | 8.9° / 40.8° |
| **o10 感知 + 10 Hz** | **100 %** | **0 次** | **1.00 / 0.92 / 1.04** | **2.4° / 3.2°** |

o2 的 163 拍 miss **100 % 发生在视线被挡的那一瞬**（`occl=True` 163/163，
`occl=False` 0）—— 也就是说：**改完感知后，"漏检"已经清零，剩下的丢失全部是
物理遮挡**。

### 22.5 遮挡期"绕视"（PEEK）—— 做了，但默认关闭

遮挡是物理的，感知再好也看不见柱子后面的人。所以加了一个状态 `PEEK`：
不把"被挡住"当"人跑了"，而是沿障碍边界绕行把视线让出来（Bug2 边界跟随），
绕通或超时后再交回需求规定的 `GOTO→SCAN`。

调试过程里有**两次方向选错**，都记在这里以免重犯：

1. 第一版按"远离障碍的一侧"横移 → 绕柱检出率反而从 73.7 % 掉到 **53.6 %**，
   距离中位 1.56 → 2.10 m、最远 4.34 m（绕的方向和人走的方向相反，越绕越远）。
2. 第二版改成"与人最后运动方向同向"的切向绕行，但绕的是**以人为中心**的圆
   （半径 = 人机距离 2.8 m）→ 6 s 只挪 0.86 m、转不到 40°，柱子还在中间。
3. 第三版改成**绕着障碍走**（圆心 = 障碍中心，半径 ≈ 障碍半径+0.63 m），方向
   取"人相对障碍的方位"一侧 → 第一次绕视 5 s 内位移 0.69 m、方向正确。

**但默认关闭（`peek_on=0`）**，原因有两个，都实测到了：

* 偶发卡滞：第二轮绕视时机器人会 5 s 静止不动（位置 0.00 位移、`w` 也不下发），
  原因未定位（怀疑是转向仲裁 owner 交接 + 安全层的组合），没查清之前不敢开。
* 收益面窄：真正"人走进障碍后面"的场景（有速度信息、有遮挡障碍）里，感知修好
  之后根本不会丢（o10 丢失 0 次）；PEEK 只在"人原地藏在柱后、机器人毫无信息"
  这种极端构造里才用得上，而这种场景需求规定的行为本来就是"前往最后位置+扫描"。

开关保留（`peek_on` / `peek_dur_s` / `peek_move_s` / `peek_v` / `peek_pass_m` /
`scan_seek_s`），查清卡滞后可开。

顺带修掉两个真 bug（与 PEEK 无关但同一批发现）：
* `_enter_scan()` 无条件刷新 `_scan_t0`，而丢失判定块每 tick 都会调它 →
  "扫描够久就横移"的判据永远不成立；
* 从 `GOTO/SCAN` 转入 PEEK 时忘了 `_nav_cancel`，两侧会同时下发 `/cmd_vel`。

### 22.6 已接入 Gazebo 的设备性能核对

| 设备 | 参数 | 结论 |
|---|---|---|
| 4 × rgbd_camera | 320×240、**5 Hz**（实测可上 10 Hz，见下）、HFOV 103°、0.1~12 m | **硬件有余量，帧率维持 5 Hz**。5 Hz 帧间 0.2 s：人 0.65 m/s → 0.13 m/帧，对正 1.5 rad/s → 17°/帧，确实偏慢。实测 10 Hz：RTF 仍 0.999、CPU 空闲 ~40 %（VMware SVGA 软件渲染、4 vCPU）→ 性能不是瓶颈；绕柱场景 \|brg\| 中位 8.9°→**2.4°**、p90 40.8°→**3.2°**、距离带 0.57~1.53→**0.92~1.04**，收益明确。**但** 10 Hz 的随机评测 3 次里出现 1 次严重退化（in_band 66.8、丢失 **11** 次、接触 **589 帧**、dmin 0.41），机理未定位 → 安全优先，**保持 5 Hz**，10 Hz 作为一行 sdf 可切、数据已留的待验证项 |
| 同上（分辨率） | 320×240 | **不动**。1.2 m 处裙摆 55×47 px 已足够；提到 480×360 要 2.25× 渲染，且 `AREA_MIN/MAX` 得同步放大 2.25 倍，收益/风险不划算 |
| front_camera | 480×360 @10 Hz（只做画面展示，不进感知） | 不动 |
| gpu_lidar | 360 线 @10 Hz、0.12~12 m、装在 0.15 m | **够用**，但只有 1 条扫描线且在 0.15 m 高：0.5 m 高的障碍能挡住裙摆却挡不住雷达，所以雷达"看得到"不代表相机"看得到" —— 这是遮挡判定必须走几何（`_los_blocked`）而不是雷达的原因 |
| 相机安装高度 | 0.25 m（chassis + 0.10） | **是根因四的放大器**，但没改：抬高要连带重标 `vis_bias_m`、视线判据和近距离几何，且 0.6 m 以上对小底盘不现实。改用"感知容错 + 行为绕行"解决 |

### 22.7 回归评测（`tmp_eval_rep.sh 3`，120 s 随机行走回放）

| 指标 | P80（上一版） | **P81（本版）** |
|---|---|---|
| in_band % | 100.0 | **100.0 / 100.0 / 100.0** |
| near / far % | 0 / 0 | **0 / 0** |
| d 中位 | 0.968 | **0.96** |
| 居中 ≤10° % | 98.7~99.3 | **99.1**（单次最低 93.0） |
| max_deg | 14.4~17.2 | **16.0**（单次离群 42.1，与 §21.7-2 同类） |
| min_d | 0.62~0.73 | **0.74** |
| 接触帧 / 事件 | 0 / 0 | **0 / 0** |
| 后退占比 | 11.2 % | **10.8 %** |
| 丢失次数 | 0 | **0** |

补记：**10 Hz 的三轮评测（p81b）** —— in_band `[66.8, 100.0, 100.0]`、
lost `[11, 0, 0]`、contact `[589, 0, 0]` 帧、w10 中位 88.8（最低 85.6）、
mx 47.2、dmin 0.41。三次里**有一次**是"反复丢失 11 次 + 撞人 589 帧"，另两次干净。
样本不足以定论 10 Hz 是否有罪（5 Hz 累积 6 次从没出现过），但**接触是安全红线**，
所以帧率维持 5 Hz，10 Hz 的结论与切换方法留在上面那张表里。

### 22.8 变更文件

| 文件 | 改动 |
|---|---|
| `work_area.sdf` | 裙摆材质变深；4 台 rgbd `update_rate` 5→10（src / install / deploy 三处同步） |
| `person_vision.py` | `AREA_MAX`→60000、`ASPECT_MIN`→0.14、`HFOV`→1.8、几何一致性改"反推可见宽度 ∈[0.09,0.60] m" |
| `follower.py` | 新增 `_los_blocked()`、`_person_vdir()`、PEEK 状态与命令分支（默认关）；`_enter_scan` 只在"新进入"时刷新 `_scan_t0`；转入 PEEK 时 `_nav_cancel`；PEEK 豁免盲走禁令；diag 增 `pk/pkd`；`_W_PRIO` 增 `peek` |
| `vla_demo/vision_config.json` | 新增 `peek_on/peek_dur_s/peek_move_s/peek_v/peek_kp/peek_pass_m/scan_seek_s` |
| 新增工具 | `tmp_orbit_test.py`（绕柱复现）、`tmp_hide_test.py`（人躲柱后）、`tmp_los_probe.py`（视线归因探针）、`tmp_det_trace.py`（离线复现检测器判决）、`tmp_snap4.py`（抓 4 相机帧）、`tmp_boot.sh`（干净重启） |

### 22.9 遗留

1. **PEEK 的偶发卡滞未定位**（第二轮绕视 5 s 静止）。查清前保持 `peek_on=0`。
2. **相机高度 0.25 m 没动**，仍是"障碍遮挡"的几何放大器。若以后允许加桅杆
   （0.45~0.55 m），0.5~0.8 m 高的障碍遮挡会明显减少，但要重标视觉偏置。
3. **`max_deg` 仍有单次 42° 离群**（§21.7-2 同一处：人贴 0.7 m 侧向走过时，
   后退让距与转向争轮速），稳态 |brg| 中位只有 2.4~2.7°。
4. **裙摆是唯一可辨特征**这一点很脆：换一身亮色衣服整套先验就失效。真要通用
   得换成"暗色裙摆 + 蓝色躯干 + 肤色头"三线索投票，或换用形状/运动特征。

---

---

## 23. 贴身死锁与设备帧率定案（P82，2026-10-26 18:00–19:10）

### 23.1 起点：p81b 那次"机理未定位"的退化

§22.6 留了一条尾巴：10 Hz 的 3 次评测里出现 1 次严重退化
（`in_band 66.8`、丢失 11 次、**接触 589 帧**、`dmin 0.41`），因为机理没查清，
安全起见把 4 台 rgbd 退回 5 Hz，10 Hz 挂为"待验证项"。

这一节就是把它查清并结掉。**结论：退化与帧率无关**，是一处控制死循环；
修掉之后 10 Hz 可以安全启用。

### 23.2 逐帧证据：不是帧率，是"后方净空"判据把自己锁死了

把 `p81b_1.json` 的 2442 条逐帧记录拉出来看 74~96 s：

```
75.05 d=0.71 det=1 TRACK  cmdv=-0.19  对正-15°(后退让距) 后退限速 后0.75
75.64 d=0.75 det=1 TRACK  cmdv=-0.45  对正+3°(后退让距)      ← 让距生效
75.94 d=0.78 det=1 TRACK  cmdv=+0.00  脱困中                ← 让距中断
77.14 d=0.51 det=1 TRACK  cmdv=+0.00  对正-38°
77.44 d=0.45 det=0 TRACK  cmdv=+0.00  对正-32°              ← 进入近距盲区
78.64 d=0.41 det=0 SCAN   cmdv=+0.00  原地扫描
81.05 d=0.42 det=1 SCAN   cmdv=+0.00  原地扫描 brg=+112°    ← 人在侧后方
82.55 d=0.41 det=1 TRACK  cmdv=+0.00  对正+113°
89.44 d=0.43 det=1 TRACK  cmdv=+0.00  后退缓退 后0.30       ← 关键
95.75 d=0.43 det=1 TRACK  cmdv=+0.00  (一直到 120s 评测结束)
```

`dist.max_streak_s = 65.15`：从 77.5 s 起贴在人身上直到评测结束。
同一时间**前方净空 `gap` 全程 1.7~2.0 m 空着没人用**。

死锁的两个半环：

1. 距离环的两处让距（`back_hold` / `close_back`）**都以"后方净空"为前提**
   （`back > lidar_stop_m + 0.25 = 0.91`）。而人绕到机器人侧后方时，
   日志里 `后0.30` 那个"障碍"**就是人自己** —— 判据永远不成立 → 永不后退。
2. 同一时刻人已进入近距盲区（`det` 时有时无，人体占满画幅），
   `back_hold` 还额外要求 `lost_dt < 0.25`；看不见人时也不退。
   而 SCAN 分支压根没有平移自由度，只会 `原地扫描`（v 恒 0）。

于是"人挡着退不了 + 看不见人不许退 + 扫描不会走"三件事同时成立，
机器人就在 0.41 m 上原地空转。10 Hz 只是碰巧走到了这个几何，5 Hz 一样会。

### 23.3 修法：新增 ESCP（贴身脱离）状态

不再试图"后退"（人挡着），而是**转到背对人的可行航向后前进脱离**，
把距离拉回可见范围再交回常规跟踪。差速底盘没有横移自由度，
"远离人"只能拆成"先转向、再前进"。

触发（放在状态机最前面，能抢占 GOTO/PEEK/SCAN/ALIGN）：

```
人距 < escape_m(0.60) 且持续 escape_delay_s(0.40)
且 后方净空 < lidar_stop_m + 0.25        ← 只有"退不动"才接管
```

第二条是刻意的：后方净空够的时候距离环自己的 `back_hold` 就把距离让开了，
ESCP 抢过去只会多一次转向。

动作：期望航向 = `atan2(ry - py, rx - px)`（背对人）→ 用 8 扇区测距在
±105° 内找一个净空 ≥ `escape_pass_m(0.55)` 的真能走的方向 → `v = 0.30`、
`w = 2.4 * 航向误差`（钳 ±1.3）→ 过 `_arb_w('escape')` 与 `_safety_gate`。
退出：人距 > `escape_free_m(0.90)` 持续 0.3 s，或超时 5 s。

配套三处（缺一不可，都是实测踩出来的）：

* **视觉"重新捕获"不能把 ESCP 拉回 TRACK** —— 否则每一拍都是
  `视觉→TRACK → ESCP` 乒乓，两处日志各刷一次、Nav2 被反复 cancel。
* **从 GOTO/SCAN 转入必须先 `pub_nav_active(False)` + `_nav_cancel()`**
  （与 PEEK 同理，否则两侧同时下发 `/cmd_vel`）。
* **ESCP 豁免"盲走禁令"**（与 PEEK 同理：脱离的前提就是看不见人）。

新增配置：`escape_on / escape_m / escape_free_m / escape_delay_s /
escape_dur_s / escape_v / escape_kp / escape_pass_m`。

### 23.4 设备核对：帧率定案 10 Hz（§22.6 的"维持 5 Hz"作废）

P81 的硬件核对结论不变：4 vCPU、VMware SVGA 软件渲染，10 Hz 时
RTF 0.999、CPU 仍有约 40% 空闲 —— **性能不是瓶颈**。真正卡住 10 Hz 的
是 §23.2 那个死循环，现在已修。

10 Hz 的收益是实打实的（绕柱受控场景 `tmp_orbit_test.py`）：

| 版本 | 检出率 | 丢失 | 距离中位 | \|brg\|中位 | \|brg\|p90 |
|---|---|---|---|---|---|
| o6　5 Hz | 100% | 0 | 1.08 | 9.3° | 40.6° |
| o10 10 Hz | 100% | 0 | 1.00 | **2.4°** | **3.3°** |

任务目标是"左右摆动不超过 ±10°"，5 Hz 的 p90 是 40.6°（超标），
10 Hz 的 p90 是 3.3°。**所以帧率改为 10 Hz。**
分辨率 320×240、gpu_lidar 360 线 @10 Hz 均不动（P81 已核对够用）。

### 23.5 绕视 PEEK 的 A/B：主场景有害，保持关闭

`peek_on` 在 §22.9 因"偶发卡滞未定位"保持 0。这一轮把它打开做了隔离对照：

| 配置 | 绕柱检出率 | 丢失 | 距离中位 | \|brg\|中位 |
|---|---|---|---|---|
| o6　5 Hz、无 ESCP、绕视关 | 100% | 0 | 1.08 | 9.3° |
| o10 10 Hz、无 ESCP、绕视关 | 100% | 0 | 1.00 | 2.4° |
| o83 10 Hz、**ESCP**、绕视关 | 99.9% | 1 | 0.96 | 4.4° |
| **o82 10 Hz、ESCP、绕视开** | **81.7%** | **24** | **1.65** | **21.8°** |

绕视一开，绕柱场景的检出率从 99.9% 掉到 81.7%、丢失 24 次、距离中位
1.65 m（最远 7.14 m）。原因：绕柱时人周期性被柱子挡住，绕视一旦触发就把
机器人往侧向带走，越绕越远 → 更看不见 → 再触发，正反馈。

**决定：`peek_on` 保持 0**，代码、参数、开关全部保留待用。

顺带试过"绕视超时后连续再绕"（`peek_rounds`）想缩短"人静止躲在柱后"的
重捕获时间，两种换边策略都比原版慢，已回退：

| 躲柱场景 | 失检合计 | 末态 |
|---|---|---|
| h82 原版（绕视开） | 16.3 s | 已重捕获 1.71 m |
| h83 逐段换边连绕 | 28.0 s | 已重捕获 1.64 m |
| h84 同向连绕 | 23.9 s | **未检出** 2.24 m |
| h85/h86 原版（绕视开） | 26.3 / 28.3 s | **未检出** |
| h87 绕视关 | 28.5 s | **未检出** 2.65 m |

即"人静止躲在柱后"目前**基本找不到**（h82 的 16.3 s 是幸运的一次），
改成连续绕视反而更糟 —— 扫描那 5 s 虽然不动，但它把 360° 重扫了一遍，
覆盖效率高于继续平移。这条留在 §23.8。

### 23.6 最终回归（10 Hz + ESCP + 绕视关）

标准评测 120 s 随机行走回放：

| 批次 | n | in_band | near | far | dmin | contact | lost | w10 |
|---|---|---|---|---|---|---|---|---|
| p81b（10 Hz、**无** ESCP） | 3 | [66.8, 100, 100] | 最大 **33.2** | 0 | **0.41** | **[589, 0, 0]** | [11,0,0] | 85.6~99.3 |
| p82a（10 Hz + ESCP） | 5 | [75.7, 100, 100, 100, 100] | **0** | 最大 24.3 | 0.77~0.84 | **0×5** | [5,0,0,0,0] | 88.8~99.3 |
| **p82c（最终，回退连绕后）** | 3 | [90.7, 100, 100] | **0** | 最大 9.3 | 0.72~0.80 | **0×3** | [2,0,0] | 91.9~99.2 |

接触帧数 589 → **0**，贴身（<0.5 m）33.2% → **0**，`dmin` 0.41 → 0.72~0.80。
`in_band` 的最差值 66.8 → 90.7（剩下那 9.3% 是纯遮挡，见 §23.8-1）。

绕柱受控场景 o83：检出率 99.9%、丢失 1 次、距离中位 0.96 m、|brg| 中位 4.4°。

### 23.7 变更文件

| 文件 | 改动 |
|---|---|
| `follower.py` | 新增 ESCP 状态（§23.3）：配置 8 项、状态字段 5 个、`§1.9` 触发/退出块、`src=='脱离'` 命令分支（8 扇区净空选向）、`_W_PRIO` 增 `escape: 2.8`、盲走禁令豁免、diag `mode='ESCP'`、视觉重捕获豁免 ESCP |
| `work_area.sdf` | 4 台 rgbd `update_rate` 5 → **10**（src / install / deploy 三处同步，md5 `067ccbb4…`） |
| `vla_demo/vision_config.json` | 新增 `escape_*` 8 项；`peek_on` 保持 0 |
| 新增工具 | `tmp_stick_test.py`（贴身死锁复现）、`tmp_orbit_test.py` / `tmp_hide_test.py` 复用 |
| 新增补丁脚本 | `patch_escp.py`（ESCP）、`patch_peek2.py` + `patch_peek3.py`（连绕试验与回退） |

### 23.8 遗留

1. **纯遮挡仍会丢 ~9%**：`p82c_1` 的 far 段实测 920/920 帧视线被 **ob1 立柱**
   挡死，机器人在柱子这侧原地转（`det=0`）而人就站在 1.88 m 外。
   绕视能救但会伤害主场景（§23.5）。可行方向：把"绕视"改成只在
   **视线被挡持续 > 3 s 且机器人已经静止**时才允许触发，用更严的闸门换掉
   现在的 `peek_on` 全开/全关 —— 这需要重做一遍 §23.5 的对照。
2. **"人静止躲在柱后"基本找不到**（26~28.5 s 内未重捕获）。属于人主动躲藏，
   超出"随机走动"的需求范围，但如果要覆盖，得引入"绕到最后已知位置的
   另一侧"这种带地图推理的行为，不是现在的启发式能做的。
3. **PEEK 的偶发卡滞（§22.9-1）仍未定位**，因为 PEEK 已确认有害而关闭，
   没有再投入；若将来要开，这条要重新过一遍。
4. **`max_deg` 仍有单次 49° 离群**（§22.9-3 同一处：人贴 0.7 m 侧向走过时
   后退与转向争轮速），稳态 |brg| 中位只有 4.4°。
5. **相机高度 0.25 m 没动**（§22.9-2），仍是遮挡放大器。
6. 评测的**人轨迹在不同轮次之间并不相同**（逐点差最大 2.33 m），
   所以"同一轮次号跨配置对比"不作数，任何结论都必须走 n≥3 的重复取样。

## 24. P83 —— 项目目录归置、依赖改造与重新编译打包

### 24.1 背景

项目根目录堆到 **946 个文件**（244 py / 232 log / 117 sh / 106 json / 98 done /
54 png …），活代码与一次性调试脚本混在一起，`deploy/` 里还留着过期版本的
`follower.py`（148 KB，比 src 里的 176 KB 旧），很容易改错文件。

### 24.2 新目录结构

```
VLA/
├── README.md          项目导航
├── vla_ws/            ROS 工作区（保留原位）
│   ├── src/vla_follow/   9 个节点 + worlds + launch + config + audio
│   ├── install/          编译产物（自包含，不用 --symlink-install）
│   └── logs/             运行日志
├── vla_demo/          运行时：启动脚本、vision_config.json、地图、语音资源
├── docs/              TaskPlan / 实施方案 / 部署文档 / architecture / reports / knowledge
├── tests/             评测链 + 受控场景 + 诊断探针 + data + out
├── tools/             organize.py / package.sh / 同步工具
├── dist/              打包产物（tools/package.sh 生成）
└── archive/           历史归档：scripts / versions / data/eval / logs / images /
                       sdf / legacy / artifacts / old-logs / packages / misc
```

归置由 `tools/organize.py` 完成（先 dry-run 再 `--apply`），共移动 **976 项**。
顶层从 946 个文件收敛到 **8 个目录 + README.md**。

**保留 `vla_ws/` 与 `vla_demo/` 原位的理由**：`install/setup.bash`、`nav2_params.yaml`、
Gazebo GUI 配置里固化了绝对路径；改名收益为零、破坏风险不小。`deploy/` 已归档到
`archive/legacy/deploy_old/`（内含过期代码副本，避免误用），分发改由
`tools/package.sh` 生成 `dist/`。

### 24.3 依赖关系改造

| 改造 | 说明 |
|---|---|
| 硬编码路径 → `VLA_HOME` | 5 个 sh 脚本顶部统一 `export VLA_HOME="${VLA_HOME:-/home/earl/workspace/VLA}"` |
| 评测链改名 | `tmp_eval_rep.sh`→`tests/eval_repeat.sh`、`p75_eval.sh`→`tests/eval_once.sh`、`tmp_run_eval.sh`→`tests/eval_run.sh`、`tmp_follow.sh`→`tests/follow_cmd.sh`、`tmp_boot.sh`→`tests/boot_demo.sh`；脚本内部相互调用全部改为 `$VLA_HOME/tests/...` |
| 场景测试改名 | `tmp_orbit/hide/stick_test.py` → `tests/test_orbit.py` / `test_hide.py` / `test_stick.py` |
| 产物统一出口 | `random_eval.py` 新增 `--outdir`，默认 `$VLA_HOME/tests/out`；三个场景测试的 jsonl 也写 `tests/out/` |
| `package.xml` 补全 | 新增 `tf2_ros`、`ros_gz_sim`、`nav2_msgs` 及 7 个 nav2_* 子包、`python3-numpy`、`python3-opencv`（原来只声明了 9 项，实际用到 20+）；版本号 0.1.0 → 0.2.0 |
| 工具根路径可配 | `tools/vmsync.py` 的 `LOCAL_ROOT`/`REMOTE_ROOT`、`sync_deploy.py` 的 `ROOT`、`traj_viz.py` 的 `--out` 全部改为环境变量/默认值 |

共用 17 处替换（`archive/scripts/patch_deps.py`、`patch_tools.py` 留存）。

### 24.4 重新编译与打包

```bash
cd vla_ws && rm -rf build install log && colcon build      # 0.86 s，1 package finished
bash tools/package.sh p83                                   # dist/VLA-20261026-p83.tar.gz
```

- 改用**非 symlink 安装**（第一次用了 `--symlink-install`，install 里的 sdf 是指向
  build 的符号链接，不利于打包分发）；改回默认后 install 自包含。
- 打包 96 MB → **3.0 MB**：主因是 `docs/knowledge/code/**/ckpt.pt`（单个 13 MB × N）
  训练权重，已在 `package.sh` 里按 `*.pt/*.pth/*.ckpt/runs/` 排除。

### 24.5 回归验证（整理后）

绕柱场景 `tests/test_orbit.py r1 70`：

| 指标 | 值 |
|---|---|
| 检出率 | 97.5 % |
| 丢失次数 | 3 |
| 距离中位 / min / max | 0.94 / 0.41 / 1.97 m |
| \|方位\| 中位 / p90 | 4.1° / 13.2° |

标准评测 `tests/eval_repeat.sh 3 p83 120`（n=3，中位数）：

| 指标 | 中位 | 最小 | 最大 |
|---|---|---|---|
| in_band | **100.0** | 100.0 | 100.0 |
| near / far | 0.0 | 0.0 | 0.0 |
| dmed | 0.97 | 0.96 | 0.97 |
| w10（±10° 内） | 98.7 | 96.5 | 99.3 |
| max_deg | 16.9 | 16.3 | 28.5 |
| dmin | 0.78 | 0.78 | 0.79 |
| contact | **0** | 0 | 0 |
| lost | **0** | 0 | 0 |

比整理前的 p82c（`in_band [90.7, 100, 100]`）更好——整理本身不改行为，差异来自
评测自身的方差，正好再次印证「单次对比不作数，必须 n≥3」。

现场 demo 重启后稳定跟随：距离 0.92~0.96 m、方位 −3°~−4°、23 个节点。

### 24.6 文档产出

- `README.md` —— 目录结构、快速开始、测试命令、当前基线指标
- `docs/部署文档.md` —— 环境依赖、安装编译、启动、参数表（距离/对正/ESCP/丢失/传感器）、
  测试方法、打包分发、9 条故障排查
- `docs/architecture/01-总体架构.md` —— 分层架构、模块职责、话题接口表、状态机转换、
  调用链、7 条关键设计决策
- 架构图/状态机图/调用链图/模块关联图（本轮对话内联呈现）

### 24.7 注意

1. **`organize.py` 会把自己也归位**（`.py` 默认进 `archive/scripts/`），已手动移到
   `tools/`；下次跑请先确认脚本位置。
2. 改代码后必须 `colcon build`——运行时用的是 `install/` 里的副本，包括 `work_area.sdf`
   （属于 `data_files`）。
3. `archive/` 不参与运行，但 `archive/data/eval/` 里的历史评测 json 是跨版本对比的
   唯一依据，不要清。

---

## 25. 双臂递水任务（P84–P92，2026-10-26 ~ 2026-10-27）

> 完整技术存档见 [`docs/reports/P84-章节.md`](reports/P84-章节.md)；
> 逐条踩坑（#19~#54）见根目录 `TaskPlan.md` §6.4；节点定义与看板见 §3 / §4。

### 25.1 需求

在已跑通的"语音驱动跟随"之上叠加双臂操作：机器人装 **2 条 7 轴机械臂 + 2 只
Shadow Hand 灵巧手**，场景里加一张桌子、一把水壶、一个带柄水杯。人说"我要喝水"
之后，机器人倒水 → 双手捧杯递出（**杯柄朝人**）→ 人喝完递回、机器人双手接回 →
杯子放回**桌面原位** → 撤离桌边 → **交还底盘给跟随，继续跟人走**。

指令链路要能手动给"人"下发"去喝水"：人收到后自己走到桌边说话，喝完继续随机活动。

### 25.2 分节点成果

| 节点 | 内容 | 验收 |
|---|---|---|
| P84 | 场景与模型（桌/壶/杯/双臂/灵巧手）、几何真源 `scene_layout.py` | `test_scene.py` 6/6 |
| P85 | 39 路关节通道与静态姿态 | `test_pose.py` 6/6 + `probe_axis_fk.py` 20/20 |
| P86 | 抓水壶（定案：世界内 `GraspCarry` 插件） | `test_p86.py` 9/9 ×3 |
| P87 | 抓杯 + 倒水（水位/水柱） | `test_p87.py` 10/10 ×3 |
| P88 | 壶放回 + 双手捧杯 + 转身面向人 | `test_p88.py` 10/10 ×3 |
| P89 | 递杯 + 人喝 + 双手接回 + 放回原位 | `test_p89.py` 10/10 ×3 |
| P90 | 人的"去喝水"指令链路（GOTO → 说话 → 语音 → 任务） | 一条命令全链 59 s |
| P91 | 端到端（一条命令 + 每阶段截图 + 结果 JSON） | `test_drink.py` 7/7 ×3 |
| P92 | 喝完水回环：撤离 → 交权 → 继续跟随 | `test_drink.py` **8/8** ×3 |

### 25.3 关键定案

1. **抓取附着 = 世界内 C++ 插件 `GraspCarry`**，不物理抓取。`DetachableJoint`
   会破坏机械臂关节约束（#19），外部 `set_pose` 时序不可控（#20）。插件是纯
   执行器，运动学只有 `arm_kin.py` 一份真源。
2. **送位一律对物体真值闭环**，不送"名义抓取位"；**松手顺序固定** 张手 → 对位 →
   松钉 → 撤手（#28，保留钉住就撤手会把物体拖走 98~127 mm）。
3. **判"没掉/没滑"用守恒量**（base 系 / 物体系），不用世界期望位 —— 后者会被
   底盘漂移污染，实测假 NG 38.5 mm vs 实际 0.0 mm（#36）。
4. **判据要双向卡**：只写下界会把"杯子被抡飞 843 mm"判成成功（#42）。
5. **撤离与交权**（P92）：`drive_to` **真开出去**（teleport 会一帧闪现），撤离点
   选在 follower 膨胀圈外；`FOLLOW` 连发 3 帧（单向话题发现期会丢消息）；演示
   脚本必须带 follower + cmd_mux + 感知三个节点，否则交了权没人接（#53）。

### 25.4 文件清单

新增：`vla_follow/gz_plugins/GraspCarry.cc`、`vla_follow/{drink_task,drink_motion,
drink_poses,carry,pngsnap}.py`、`tools/{gen_drink_scene,start_scene,start_drink_demo,
gui_cam,check_ik}.py`、`vla_demo/send_person_cmd.sh`、`tests/test_{scene,pose,p86,p87,
p88,p89,drink}.py`、`tests/diag/diag_p8{8,9}_*.py`、`docs/reports/P84-章节.md`

改动：`vla_follow/{follower,person_walk,voice_node,cmd_mux,gen_map,scene_layout}.py`、
`launch/demo.launch.py`、`config/bridge.yml`、`README.md`、`docs/部署文档.md`、
`docs/architecture/01-总体架构.md`

### 25.5 注意

1. 改布局只改 `vla_follow/scene_layout.py`，然后 `gen_drink_scene.py` →
   `check_ik.py` → `test_drink.py`；不要手改 SDF 里由脚本生成的那一段。
2. 端到端挂了先用 `test_p86/p87/p88/p89.py` 分段二分，比反复跑端到端快得多。
3. `drink_task` 是 39 路关节指令与抓取指令的**唯一发布者**；测试脚本要用
   `DrinkTask(observer_only=True)` 只读观察，否则两个节点抢发布者。
