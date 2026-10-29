
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
