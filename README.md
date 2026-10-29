# VLA —— 语音驱动的人形跟随 + 双臂递水演示

**能力一 · 跟随**：人在场地里随机走动、转向、停顿、后退，机器人用 4 向
RGB-D 相机找到他，把他保持在 front 相机视野中间（左右摆动 ≤10°）、距离稳定在
1.0 m 附近；人走出全部视野后，按最后 10 次抓拍推测位置前往，到达后扫描重找；
目标重现立即重新对正。全程优先避障，绝不接触。

**能力二 · 喝水任务（双臂 + 灵巧手）**：机器人装 2 条 7 轴机械臂 + 2 只 Shadow
Hand 灵巧手，场景里有一张桌子、一把水壶、一个带柄水杯。人说一句"我要喝水"
（或手动给"人"下发指令让他走过去说），机器人就：

```
抓壶 → 倒水入杯（水位可见上升）→ 壶放回原位
→ 双手捧杯转身面向人（杯柄朝人，方便接）→ 前伸递到人手边
→ 人接杯喝 → 双手接回 → 转回桌前 → 杯子放回原位
→ 双臂收回、撤离桌边 → 把底盘交还给跟随 → 人走开，机器人跟上去
```

技术栈：ROS 2 Jazzy + Gazebo Harmonic + Nav2 + Vosk 离线中文语音 + KDL 逆解。

---

## 目录结构

```
VLA/
├── README.md              本文件
├── vla_ws/                ROS 2 工作区（colcon）
│   ├── src/vla_follow/    唯一的功能包：14 个节点 + 世界 + launch
│   │   ├── gz_plugins/    世界内 C++ 插件 GraspCarry（抓取/附着/关节运动学驱动）
│   ├── install/           编译产物（自包含，可打包）
│   └── logs/              运行日志（gzsim.log / demo.log）
├── vla_demo/              运行时目录
│   ├── start_demo.sh      一键启动（单实例互斥，先起 gz 再起 ROS）
│   ├── stop_demo.sh / send_voice.sh / show_follow.sh
│   ├── send_person_cmd.sh 给"人"下指令：drink / walk / stop
│   ├── vision_config.json 全部可调参数（跟随距离/PID/避障/状态机）
│   ├── gz_gui_fix.py      启动前重写 Gazebo GUI 布局
│   ├── traj_viz.py        轨迹可视化
│   ├── gen_map 产物 maps/  Nav2 静态地图
│   └── vosk-model-*/      离线语音模型
├── docs/                  文档
│   ├── TaskPlan.md        开发日志（P0~P82 全部问题与修复）
│   ├── VLA实施方案.md     原始方案
│   ├── 部署文档.md        环境搭建、编译、启动、排障
│   ├── architecture/      架构图与流程说明
│   ├── reports/           各阶段诊断报告
│   └── knowledge/         VLA 领域知识库（14 篇）
├── tests/                 测试与评测
│   ├── boot_demo.sh       干净重启 demo
│   ├── eval_repeat.sh     标准评测 ×N 次取中位（推荐 n≥3）
│   ├── eval_once.sh       单次评测（清场 + 起 demo + 跑）
│   ├── eval_run.sh        评测执行器（调 random_eval.py）
│   ├── random_eval.py     随机行走综合评测（含回放模式）
│   ├── test_orbit.py      绕柱场景：人绕立柱转圈
│   ├── test_hide.py       躲柱场景：人静止在柱子背后
│   ├── test_stick.py      贴身死锁场景：人被固定在机器人正后方
│   ├── test_scene.py      P84 场景/模型验收（6 项）
│   ├── test_pose.py       P85 关节通道与静态姿态（6 项）
│   ├── test_p86.py ~ test_p89.py  抓壶 / 倒水 / 捧杯 / 递接放回 分段验收
│   ├── test_p99.py        腕部 RGB-D（眼在手）验收（6 项）
│   ├── test_p101.py ~ test_p102.py  动作拟人化（复位/肘朝外/双手放回、
│   │                        先退到位再垂手 / 站远抬臂再驶入）验收
│   ├── test_p103.py       全程关节姿态是否符合人体运动规则（7 项）
│   ├── test_drink.py      P91+P92 端到端（一条命令跑完整条链，8 项）
│   ├── diag/              诊断探针（抓图/亮度扫描/检测判决回溯等）
│   ├── data/walk_rec.jsonl 评测用的人行走轨迹回放
│   └── out/               测试产物（json / log / png），运行时生成
├── tools/                 运维工具
│   ├── package.sh         打包分发
│   ├── vmsync.py / vmput.py  本地 ↔ 远端同步（SFTP）
│   ├── vmpush.py           走 SSH exec 的推送（SFTP 在本项目家目录会 ENOENT）
│   ├── sync_deploy.py     部署目录同步
│   ├── gen_drink_scene.py 生成喝水场景 SDF（坐标取自 scene_layout.py）
│   ├── start_scene.sh     起 gz 世界 + 桥接（--gui 带窗口）
│   ├── start_drink_demo.sh  起喝水演示节点组（感知/follower/cmd_mux/人/语音/任务
│   │                         + P97 起 默认带 gt_tf/Nav2/gz_viz，NAV2=0 退回）
│   ├── tp.py                命令行摆位助手（调试）
│   ├── reset_props.py       壶/杯复位（松钉→重建→换绑，含真值校验）
│   ├── truth_probe.py       一次性真值探针（robot/person/kettle/cup/液面体）
│   ├── open_gui.sh / gui_cam.py  Gazebo 窗口与机位预设
│   ├── wrist_cam_shot.py   腕部相机抓帧落 PNG（走 gz 原生订阅）
│   └── check_ik.py        离线可达性体检（改布局后先跑它）
├── dist/                  打包产物（由 tools/package.sh 生成）
└── archive/               历史归档，不参与运行
    ├── scripts/           一次性调试脚本（原 tmp_*/ patch_*）
    ├── versions/          各版本 follower 快照
    ├── data/eval/         历史评测结果（跨版本对比用）
    ├── logs/  images/  sdf/  legacy/  …
```

## 快速开始

```bash
export VLA_HOME=/home/earl/workspace/VLA    # 项目根，所有脚本从这里解析

# 1) 编译（首次或改动代码后）
cd $VLA_HOME/vla_ws && source /opt/ros/jazzy/setup.bash && colcon build

# 2) 启动 demo（干净重启，seed=7）
bash $VLA_HOME/tests/boot_demo.sh 7

# 3) 下发跟随指令
bash $VLA_HOME/tests/follow_cmd.sh

# 4) 看跟随状态
grep FOLLOWER $VLA_HOME/vla_ws/logs/demo.log | tail -5
```

## 启动 Gazebo（世界 / 窗口 / 视角）

分三层，按需取用；日志都在 `vla_ws/logs/`（服务端 `gzsim.log`、GUI `gzgui.log`）。

**① 只起世界（服务端，无窗口）**

```bash
bash $VLA_HOME/tools/start_scene.sh     # 清场 → gz 世界 → ros_gz_bridge
```

内部核心命令（手动等价写法）：

```bash
gz sim -r -v 3 --render-engine ogre -s \
    $VLA_HOME/vla_ws/install/vla_follow/share/vla_follow/worlds/work_area.sdf
```

- `-r` 起来即运行；`-v 3` 日志级别；`-s` 仅服务端
- `--render-engine ogre`：本机 VMware SVGA 软件光栅下 ogre2 黑屏，必须回落

**② 开观察窗口（GUI 客户端，挂到已在跑的世界，需 `DISPLAY=:0`）**

```bash
bash $VLA_HOME/tools/open_gui.sh free    # 自由视角（全屏 3D，推荐）
bash $VLA_HOME/tools/open_gui.sh wide    # 全场机位 + 左上角相机小窗
bash $VLA_HOME/tools/open_gui.sh close   # 机械臂特写机位
bash $VLA_HOME/tools/open_gui.sh top     # 俯视全景（看导航轨迹）
```

其核心命令就是 `gz sim -g`；机位写在 `~/.gz/sim/8/gui.config`。
`free` 预设会摘掉相机小窗、全屏留给 3D 视图：**左键拖动=环绕旋转、
中键拖动=平移、滚轮=缩放**。

`open_gui.sh` 默认 `BIG=1`：把 3D 视图撑到整屏、把 Component inspector /
Entity tree 收成 2×2 px 的浮动面板，窗口尺寸按 `xdpyinfo` 实际分辨率算，
最后交给 `tools/gui_arrange.sh` 落位。窗口管理器（mutter）会把新映射的
窗口"重新摆放"，`wmctrl -e` 发一次往往被覆盖 —— `gui_arrange.sh` 会读回
几何反复校正，**别再用 `xdotool search --name` 拿窗口 id**：它同时命中
框架窗口和客户区窗口，`head -1` 经常抓到框架，resize 一点效果都没有
（用 `wmctrl -l` 的 id 才对）。

**②' 手腕/掌侧相机预览（边跑边看四路 RGB-D）**

```bash
bash $VLA_HOME/tools/wrist_view.sh          # 2×2: 左/右手 × RGB/深度
bash $VLA_HOME/tools/wrist_view.sh 3        # 放大 3 倍
bash $VLA_HOME/tools/wrist_view.sh stop     # 关掉
```

窗口里按 `1` 切手腕相机（`/wrist_cam_*`）、`2` 切掌侧相机（`/side_cam_*`，
P100 —— 这一路才看得见要抓的东西），`q`/`d`/`s`/`+`/`-` 切源与缩放。
预览是 5 Hz 且**只在有新帧时重画**：不加这个门，GUI + 预览会把 RTF 从
1.00 拖到 0.78（4 核 VM）。

**②'' 让 Gazebo 机位跟着机器人（围观用）**

```bash
python3 $VLA_HOME/tools/gui_follow.py robot        # 近距离跟拍
python3 $VLA_HOME/tools/gui_follow.py robot 0.62   # 距离再拉近 38%
```

**③ 世界 + 全部业务节点（完整演示，含 Nav2）**

```bash
bash $VLA_HOME/tools/start_scene.sh          # 世界 + 桥接
bash $VLA_HOME/tools/start_drink_demo.sh     # 感知/follower/cmd_mux/人/语音/任务
                                             # + gt_tf/Nav2/gz_viz（NAV2=0 退回）
```

## 喝水演示（双臂递水）

```bash
# 1) 起 gz 世界 + 桥接（--gui 会同时打开 Gazebo 窗口）
bash $VLA_HOME/tools/start_scene.sh --gui

# 2) 起喝水演示节点组（感知 / follower / cmd_mux / 人 / 语音 / 喝水任务）
bash $VLA_HOME/tools/start_drink_demo.sh
```

**默认不用管它**：机器人平时一直在跟随人；"人"自己会随机想喝水（首次 45 s
后，之后每隔 90~180 s），想喝了自己走到桌边说"我要喝水"，机器人让位去做
倒水 / 递水 / 接回 / 放回，做完撤离桌边、把底盘交还 follower，继续跟着人走。

```bash
# 想立刻看一次（替他决定"现在去喝水"）
bash $VLA_HOME/vla_demo/send_person_cmd.sh drink
```

随机喝水的行为可调（都是 `person_walk` 的环境变量）：

| 变量 | 默认 | 说明 |
|---|---|---|
| `PW_AUTO_DRINK` | `1` | 设 `0` 关掉随机喝水，只留手动下发 |
| `PW_DRINK_FIRST` | `45` | 启动后第一次"想喝水"的延迟（秒） |
| `PW_DRINK_GAP_MIN` / `PW_DRINK_GAP_MAX` | `90` / `180` | 两次喝水之间的随机间隔（秒，从**喝完**起算） |
| `PW_DRINK_JITTER` / `PW_DRINK_JITTER_DEG` | `0.06` / `8` | 人每次站的位置偏差（米 / 度）；设 `0` 则严格站到接水位 |

```bash
# 例：关随机喝水（跑 P94 扰动验收时必须关，否则自发行为会和验收指令打架）
PW_AUTO_DRINK=0 bash $VLA_HOME/tools/start_drink_demo.sh
```

想直接驱动机器人（跳过"人走过去说话"这一步）：

```bash
ros2 topic pub --once /robot/command std_msgs/msg/String \
  'data: "{\"action\":\"DRINK\"}"'
```

全流程约 2.5~3.5 分钟，阶段状态实时广播在 `/drink/state`：

```
IDLE → PREPARE(倒水+捧杯+转身) → PRESENT(前伸递杯 + 人双手张开迎)
→ HANDOVER(人双手合拢捧住 → 交到人手) → PERSON_DRINK(人举杯喝)
→ TAKE_BACK(人双手张开交还 → 机器人双手接回 → 人收手)
→ PUT_BACK(放回原位) → RETREAT(撤离桌边) → HANDBACK(交还底盘给跟随)
→ DONE → IDLE
```

分阶段验收脚本（定位问题时用它二分，比端到端快）：

```bash
python3 tests/test_p86.py    # 抓水壶
python3 tests/test_p87.py    # 抓杯 + 倒水（水位/jet）
python3 tests/test_p88.py    # 壶放回 + 双手捧杯 + 转身面向人
python3 tests/test_p89.py    # 递杯 + 人喝 + 双手接回 + 放回
python3 tests/test_p96.py    # 人的双手：张开迎杯 / 合拢捧住 / 张开交还
python3 tests/test_p99.py    # 腕部 RGB-D：出流 / 桥接 / 外参 / 桌面反投
python3 tests/test_p103.py   # 全程关节姿态是否符合人体运动规则（倒水左肘内收）
```

"人"的两条手臂是**各自一条自由度**（绕肩点的竖直轴外展），所以接杯这段是真
的动作而不是摆好姿势等杯子被钉进来：手心间距 **184mm（合拢，正好贴住杯壁）
↔ 345mm（张开，杯子两侧各留 81mm）**。两侧关节在 SDF 里同轴、上层发**反号**
值，杯子则始终钉在肩架上（两只手各拽一半是不行的 —— 抓握附着只允许一个宿主）。

## 测试

```bash
# 标准评测：120 s 随机行走 ×3 次，输出中位数（推荐，单次波动可达 ±6pt）
bash tests/eval_repeat.sh 3 mytag 120

# 受控场景（需先启动 demo）
python3 tests/test_orbit.py r1 70      # 绕柱：检出率 / 丢失次数 / 方位偏差
python3 tests/test_hide.py  h1 35      # 躲柱：重捕获耗时
python3 tests/test_stick.py s1 25      # 贴身：ESCP 能否拉开距离
```

产物统一落在 `tests/out/`。

## 关键指标（当前基线）

| 指标 | 口径 | 当前值 |
|---|---|---|
| 距离合格占比 in_band | 0.5~1.5 m | 100 % |
| 最小距离 dmin | 单轮最小 | 0.75 m |
| 方位在 ±10° 内 w10 | 检出帧 | 98.3 % |
| 物理接触 contact | 帧数 | 0 |
| 丢失 lost | 次数 | 0 |

> 基线为 `tests/eval_repeat.sh 3 p84 120` 三轮中位数（桌子进障碍表后的回归，
> 2026-10-27）：in_band 100 %、dmed 0.95 m、dmin 0.75 m、w10 98.3 %、
> contact 0、lost 0 —— 与加桌子之前持平。

### 喝水任务（`tests/test_drink.py` 9/9）

| 指标 | 口径 | 当前值 |
|---|---|---|
| 人到位偏差 | 走到接水位 | 87.9~90.0 mm（面向误差 7.7~7.9°） |
| 倒水 | 水位 / 水柱长度 | 65 % / 159.9 mm |
| 递杯 | 杯心到位 / 到人的手 | 5.0~14.5 mm / 12.9~24.8 mm |
| 交接守恒 | 交接瞬间漂移 | 0.0 mm |
| 杯子放回原位 | 与原位偏差 / 杯轴倾角 | 0.7~1.5 mm / 0.02° |
| 撤离 → 交权 | RETREAT / HANDBACK 阶段 | 齐全（真开出去 1.15 m） |
| 恢复跟随 | 60 s 内行驶 / 末段人机距离 | 10.6~11.3 m / 0.91~0.96 m |
| 人的双手 | 手心间距 张开→合拢 | 345 → 184 mm |
| 人的双手 | 手到杯轴（相切 92 mm） | 交接前 88.3/97.5 → 交接后 88.3/97.5 mm（变 0.0） |
| 人的双手 | 交还时手到杯轴 | 169 / 179 mm |
| 全流程 | 一条命令到放回 + 跟上 | 129~153 s（随人起点距离） |

### 收臂 / 入场顺序（`tests/test_p102.py` 8/8）

用户 2026-10-28 的两条"像人一样"的**顺序**要求，都只用关节轨迹 + 闭环 IK 实现：

1. **放回水杯后**先退到臂长之外，再从前方自然垂手：`hang_path()` 分三段 ——
   ①只折肘（肘点是支点，世界位移 0 mm）→ ②肩部三轴归零 → ③伸肘到与上臂平行。
   松钉到倒车之间还有一步 `hands_off()`（两手先后撤 120 mm、再抬 90 mm），
   否则掌心贴着杯壁倒车会把杯子带偏（实测 47.9 mm）。
2. **去倒水前**站远抬臂：`pour_approach()` —— 先在 gap≈0.80 m 处把前臂折起、
   双手抬到器物高度（掌心 z=0.82 m），再闭环驶入服务位（残差 23 mm）。

| 项 | 判据 | 3 次实测 |
|---|---|---|
| 撤手/倒车/收臂不碰杯子 | 杯子水平漂移 ≤ 15 mm | **0.0 mm**（三段各 0.0） |
| 收臂**时**车离桌沿 | ≥ 臂+手全长 0.715 m | L **1.345~1.349** / R **1.308~1.320** m（余量 +0.57 m） |
| 前臂绕**肘**回收 | 肘点世界位移 ≤ 5 mm | **0.0 mm**（19~21 个路点，折肘 −1.75 rad） |
| 收完悬垂 | 关节偏差 ≤ 3° | **0.0° / 0.0°**，实测 max\|q\| **0.0** rad |
| 全程无翻转 | 腕部**单步**变化 ≤ 30° | L **14.6~15.2**° / R **16.1~16.3**° |
| 抬臂发生在臂长之外 | `gap_before` ≥ 0.715 m | **0.805~0.810** m |
| 双手抬到器物高度 | IK 残差 ≤ 5 mm | L **1.4~4.7** / R **0.4~4.1** mm |
| 抬完才驶入 | gap ≤ 0.30 m 且行驶残差 ≤ 30 mm | **0.21 m / 23.2~23.8 mm** |

### 关节姿态的人体规则（`tests/test_p103.py` 7/7 连跑 3 次）

用户 2026-10-28：**"倒水的时候，机器人左手臂肘部弯向内侧，这与人体骨骼运动规则
不符合"**。病根不是"某个关节"，是 **7 轴臂对 6 维位姿冗余的那一维没人管** —— 同一
只手掌到同一处位姿有一整族等价解，逆解只按种子收敛、只挑残差最小，于是挑到

```
举杯(倒水) L  q = [0.12, −0.61, −0.41, −1.64, −0.32, +0.71, −2.67]
肘只外扩 22mm、前臂反拧 171°、肘垂在掌下 219mm，而闭环报"到位 1.7mm"
```

六条规则（`arm_kin.violation_list`，运行时日志与验收脚本同源）：

| 规则 | 内容 | 阈值 |
|---|---|---|
| R1 | 肘反关节 | `elbow ≤ 0.02` |
| R2 | 肘越过身体中线 | `sgn·肘y ≥ −5mm` |
| R4 | 肩内旋过大 | `\|up_roll\| ≤ 2.00`（= 机械限位） |
| R5 | 前臂反拧 | `\|fore_roll+wrist_r\| ≤ 2.00`（115°） |
| R6 | 腕反折 | `\|wrist_p\| ≤ 1.50`（= 机械限位） |
| R7 | 肘过度折叠 | `\|elbow\| ≤ 2.25`（129°，人的屈曲上限 145°） |

修法是给冗余维定**策略**（`arm_kin.ik_human`）：pan×lift×up_roll×elbow **324 组
种子**（只在进入一个动作时算一次，≈0.9s）各解一遍，按"越限 + **贴限** + 内收 +
关节应力 − 肘外扩偏好"的代价挑最小，并且**一个动作只决策一次**（迟滞 + 代价闸）。

| 项 | 判据 | 实测 |
|---|---|---|
| 举杯时左肘朝外 | out_line ≥100 / droop ≤120 / \|twist\| ≤2.2 | **210~223** / **−6~+1** / **−1.61~−1.64**（老实现 **49 / 219 / −2.94**） |
| 倒水时右肘朝外 | out_line ≥60 | **163~170** mm |
| 倒水段两侧 | 六条规则 0 违反 | 全过 |
| 捧杯 / 放回 / 撤手 / 悬垂 | 六条规则 0 违反 | 全 0；捧杯外扩 L **224~244** / R **180~183** mm |
| 全流程 trace | 5 个检查点 0 违反 | 全 0 |
| 收尾悬垂 | max\|q\| ≤0.10 | L **0.0** / R **0.0** rad |

右臂原来几个"肘贴着身体"的位形被一起修好：持壶 **93→145**、放壶 **38→113**、
捧杯 **38→179**、递杯 **29→137** mm（离线 11 个位形 0 违反）。

### 双手接杯（`tests/test_p96.py` 10/10）

人的两条手臂拆成各自的 link 后，"双手接杯"才有可验收的对象。全部量**真值**
（两条手臂 link 的世界位姿）—— 人的关节不在 `/drink/joint_state` 里，读不回来。

| 项 | 实测 |
|---|---|
| 开合幅度与几何一致 | θ=0.035/0.22/0.42 → 手心间距 184.1 / 260.7 / 345.3 mm，与几何预测差 **0.00 mm**，左右不对称 **0.00 mm** |
| 张开迎杯 | 手心间距 345 mm，张开时杯子被碰动 **0.0 mm** |
| 合拢捧住 | 手到杯轴 **89.0 / 95.8 mm**（相切 92±25，左右差 6.7 mm） |
| 交接 | 杯子漂移 **0.0 mm**、交接前后手-杯间距变化 **0.0 mm** |
| 人举杯喝 | 抬高 172.4 mm（理论 177），双手仍捧着 88.8 / 95.5 mm |
| 张开交还 | 手到杯轴 89/96 → **170/177 mm**，间距回到 184 mm |

判据：手到杯轴 **92±25 mm**、左右差 <50 mm、手与杯高差 <80 mm。25 mm 的来历是
手掌 r52 + 杯 r40 —— 偏 25 mm 就是一侧陷进杯壁 25 mm、另一侧留 25 mm 缝，再大
就看得出"一只手插进杯子里、另一只没碰到"了。

### 腕部 RGB-D（眼在手，`tests/test_p99.py` 6/6）

两只手的**手背**各挂一路 `rgbd_camera`（256×192 @5Hz，一路传感器同时出 RGB 与
32FC1 深度，已桥到 ROS 的 `/wrist_cam_{R,L}/{image,depth_image}`）。机位/朝向只
在 `arm_kin.WRIST_CAM_*` 一处定义，SDF 由它生成、验收脚本也用它把世界点投到图上。

```bash
python3 tests/test_p99.py                     # 6 项验收（出流/帧率/桥接/外参/桌面反投/截图）
python3 tools/wrist_cam_shot.py wrist         # 随时抓一帧落 PNG（走 gz 原生订阅）
python3 tests/diag/diag_wrist_proj.py R       # 光轴/像素轴向的标定探针（改机位后先跑它）
```

标定怎么自证：由**实测关节角 FK** 出四个指尖投到图像，与深度图"最近一片"的质心
差 **20 px**；再把整幅深度图反投回世界，落在桌面高度的点拟合出 `z=0.721`（真值
0.72）、倾角 **0.2°**。两条都对上，才敢说"看到的位置就是算出来的位置"。

**用途与边界**：它是**接近相机**——看手前方的桌面、手指与工作区的相对位置；在
0.2 m 内看不到被抓物（被抓物就在指尖那一侧，被自己的手指挡死），要"抓取特写"
得换侧挂机位或更广角镜头，不是调参能解决的。细节见 `TaskPlan.md` P99 与 §7.1。

### 接水位扰动鲁棒性（`tests/test_robust.py`，P94）

给"去喝水"的目标点加 ±0.15 m / ±20° 的扰动，看递杯还能不能接上。核心指标是
`d_hand` = 递杯阶段**杯子到人手的水平间距**——它决定了交接那一刻杯子会不会瞬移。

| 组 | 扰动 | 到位偏差 | 离名义位 | 杯-手间距 | 放回偏差 |
|---|---|---|---|---|---|
| base | 无（对照） | 91 mm | 91 mm | **2.1 mm** | 1.5 mm |
| far | 远离机器人 0.15 m | 87 mm | 218 mm | **1.0 mm** | 0.1 mm |
| near | 靠近机器人 0.15 m | 90 mm | 122 mm | **2.5 mm** | 1.5 mm |
| side+ | 切向 0.15 m | 102 mm | 251 mm | **2.3 mm** | 1.7 mm |
| side- | 切向 0.15 m | 84 mm | 102 mm | **1.4 mm** | 0.7 mm |
| yaw- | 朝向 −20° | 93 mm | 93 mm | **0.6 mm** | 5.1 mm |
| yaw+ | 朝向 +20° | 88 mm | 88 mm | **0.3 mm** | 1.7 mm |

判据：到位 <150 mm、杯-手 <45 mm、放回 <10 mm。**7/7 组全项通过**，最差一组
（side+）杯-手间距也只有 2.5 mm。

兜底链路：人站偏 → 转身按**实测人位** → 臂前伸上限 0.62→**0.70 m**（离线扫出来的
安全上限）→ 还够不着就**底盘朝人补一段**（贴桌距离用桌子矩形算，不用外接圆；
前进速度下限降到 0.02 m/s 并按实测多补一轮 —— 原来 30 mm 的固有残差会把
38 mm 的补位几乎全吃掉）→ 递杯方向始终指向**实测人手**。

## 文档

- [部署文档](docs/部署文档.md) —— 环境、编译、启动、配置、排障
- [架构说明](docs/architecture/01-总体架构.md) —— 分层架构、状态机、调用链
- [TaskPlan](docs/TaskPlan.md) —— 完整开发日志（每个问题的现象、根因、修法）
