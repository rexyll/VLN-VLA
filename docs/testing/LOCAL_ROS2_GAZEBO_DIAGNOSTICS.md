# 本地 ROS 2 Jazzy + Gazebo Harmonic DrinkTask 测试与日志采集

本流程用于在 Ubuntu 24.04 虚拟机执行真实 Gazebo 生产 work_area 场景。日志采集器是只读旁路节点，不发布控制指令，不改变控制回路、物理参数或验收阈值。

## 1. 获取代码

只使用当前开发分支，不切换 main，也不创建新分支：

```bash
git clone https://github.com/rexyll/VLN-VLA.git
cd VLN-VLA
git fetch origin
git checkout experiment/drinktask-bt-ik-recovery
git pull --ff-only origin experiment/drinktask-bt-ik-recovery
```

已有仓库则执行：

```bash
cd /path/to/VLN-VLA
git status --short --branch
git fetch origin
git checkout experiment/drinktask-bt-ik-recovery
git pull --ff-only origin experiment/drinktask-bt-ik-recovery
```

如果工作区有未提交修改，先备份并确认，不要直接覆盖。

## 2. 检查环境与编译

```bash
lsb_release -a
echo "ROS_DISTRO=$ROS_DISTRO"
gz sim --version
source /opt/ros/jazzy/setup.bash
cd vla_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

应使用 Gazebo Harmonic / Gazebo Sim 8，与项目现有插件 ABI 保持一致。不要混用 Fortress、Garden 或其他 Gazebo major 版本。

确认 MCA 组件已经包含在当前工作区：

```bash
ros2 pkg prefix mca_drink_control
ros2 interface show mca_drink_control/srv/SolveIk
```

若包或接口不存在，先停止测试并保留完整构建输出；不要从另一个仓库临时复制浮动版本。

## 3. 启动前自检

```bash
source /opt/ros/jazzy/setup.bash
source vla_ws/install/setup.bash
mkdir -p tests/out
python3 - <<'PY'
import rclpy
import gz.transport13
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.model_pb2 import Model
print("ROS 2 and Gazebo Python bindings: OK")
PY
```

不要在同一仿真世界同时启动两套 drink_task、两套 cmd_mux 或多个命令发布器。

## 4. 启动生产场景与日志采集

终端 A：Gazebo 生产世界和 bridge：

```bash
source /opt/ros/jazzy/setup.bash
source vla_ws/install/setup.bash
bash tools/start_scene.sh --gui
```

等脚本确认 /world/follow_world/chassis_lock/state 已注册，且没有插件加载错误后再继续。topic 存在只说明插件已注册，不代表物理锁已通过动作验收。

终端 B：只读日志采集器。先启动并在整轮测试期间保持运行：

```bash
source /opt/ros/jazzy/setup.bash
source vla_ws/install/setup.bash
python3 tests/diag/record_drinktask_telemetry.py --run-id local-001
```

采集器会输出目录，例如 tests/out/drinktask_local-001_YYYYMMDD_HHMMSS/，内含：
- metadata.json：系统、ROS、分支、HEAD、订阅情况，以及从 `bridge.yml` 自动解析出的 39 路 `/drink/cmd/*` ROS→Gazebo topic 映射和订阅注册结果。
- telemetry.jsonl：每秒（在 10Hz 记录流中按需写入）额外记录 ROS graph snapshot，覆盖 39 路命令 topic 及 `/drink/state`、`/drink/base_lock_state`、`/drink/cmd_vel`、`/robot/command`、`/robot/cmd_vel` 的发布者数量、节点名/命名空间和类型。该信息用于区分“没有发布者”与“有发布者但没有消息”。
- telemetry.jsonl：10Hz 快照，包含 ROS 关节位置/速度/effort、Gazebo truth 底盘位姿、Gazebo 关节遥测、物理锁状态、软件底盘锁 ACK、底盘指令、odom、DrinkTask 状态、follower/Nav2 状态、TF 和 topic 数据新鲜度；另逐路记录 39 个命令 topic 在 ROS 端与 Gazebo Transport 端的实际接收计数、最后值、最后接收 UTC/monotonic 时间戳和数据 age。

必须成功订阅以下三条 Gazebo topic：dynamic_pose、robot joint_state、chassis_lock state。若订阅失败，先不要开始正式验收，把报错和 topic 列表发回分析。

终端 C：业务节点：

```bash
source /opt/ros/jazzy/setup.bash
source vla_ws/install/setup.bash
bash tools/start_drink_demo.sh
```

确认 mca_ik_server、drink_task、cmd_mux、follower 等日志没有启动错误，再触发一轮：

```bash
bash vla_demo/send_person_cmd.sh drink
```

每次只触发一轮，等待完整终态后再开始下一轮。

## 5. 测试顺序

第一次不要连续跑多轮，按顺序执行：

1. 启动与锁止握手：检查 attached → detached → attached → 最终 detached 的状态转换、位姿变化和 mux LOCKED ACK。
2. 启动 home：检查左右臂和手指的目标/实测位置、速度、effort 是否有有效数据，并确认达到现有到位门槛。
3. 单轮完整 DrinkTask：保留 PREPARE、PRESENT、HANDOVER、PERSON_DRINK、TAKE_BACK、PUT_BACK、RETREAT、HANDBACK、DONE 或明确 FAIL 状态。
4. 若单轮成功，再跑第二轮检查状态重置与重复性。失败后不要自动重试或跳过失败阶段。

不要修改关节限位、质量/惯量、摩擦、底盘物理锁、抓取/交接阈值、IK/collision 门槛，也不要用 teleport 或伪造真值制造通过。

## 6. 停止与整理日志

测试完成后，在终端 B 按 Ctrl-C 正常停止采集器，然后停止业务节点和 Gazebo。打包本轮完整输出目录：

```bash
tar -czf drinktask_local-001_logs.tar.gz -C tests/out <实际生成的目录名>
```

同时保存以下原始日志（如果存在）：
- vla_ws/logs/gzsim.log
- vla_ws/logs/bridge.log
- vla_ws/logs/drink.log
- vla_ws/logs/mca_ik.log
- vla_ws/logs/mux.log
- vla_ws/logs/follower.log
- vla_ws/logs/nav2.log
- 完整 colcon build 输出（若本轮重新编译）

上传压缩包到项目指定日志目录，并告知 run-id、是否完成整轮、终态和观察到的现象。不要只上传截断后的 grep 结果。

## 7. 分析重点

- 底盘是否真的锁住：Gazebo chassis truth pose/yaw/twist、wheel joint velocity/effort、/drink/cmd_vel、/robot/cmd_vel、物理 DetachableJoint state、软件 mux lock ACK。
- 机械臂命令是否到达执行端：/drink/joint_state 与 Gazebo joint state 的 position/velocity/effort，结合 MCA 服务请求/结果日志。
- 杯/壶/人/机器人相对关系：drink.log 中的 PRESENT_EXEC_RESULT、PRESENT_FOLLOW_PRE/POST、grasp/place 和阶段验收记录，以及 Gazebo truth 采样。
- 时序与数据有效性：topic age、TF、odom 与 Gazebo truth 一致性、采样中断。
- 失败定位：以首个 FAIL_* 状态或第一个超限时刻为主，不把后续级联错误误判为根因。

采集器只负责连续遥测；项目已有动作级日志、异常 traceback、JSON 测试报告和截图仍须一并保留。
