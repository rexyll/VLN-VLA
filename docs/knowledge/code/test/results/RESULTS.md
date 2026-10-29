# VLA × Gazebo 仿真部署验证报告

> 验证日期：2026-10-23 ｜ 环境：VMware 虚拟机 192.168.255.128（Ubuntu 24.04.5，4C/8G，Gazebo **Harmonic gz-sim 8**，headless：Xvfb + 软件渲染 llvmpipe）

## 1 部署清单（全部在虚拟机上完成）

| 项 | 状态 |
| --- | --- |
| C++ 系统插件 `vla_bridge.cpp` 编译（gz-sim8 API） | ✅ BUILD_OK（g++ + pkg-config gz-sim8） |
| 世界 `vla_arena.world` 部署（地面/智能体/绿箱/双十字/灯光） | ✅ gzserver 长稳运行 |
| **虚拟相机** 顶视 64×64（视场 64cm×64cm，1px=1cm） | ✅ 标定误差 **0.67px**（亚像素） |
| **虚拟设备** 侧视监控相机 / 接触代理 / 速度积分智能体 | ✅ `/gz_side_cam`、STATE.contact 字段 |
| Python 侧 `gz_env.GzArenaEnv`（与 mini_vla 同机器人接口） | ✅ 训练帧与实时帧逐像素一致（diff=0） |
| 数据采集（脚本专家）→ 域内训练 → 闭环部署 全链路 | ✅ 587 集成功演示 / 4361 有效帧 |

## 2 三个 Demo 的验证结果（最终模型 v7）

| Demo | 验证内容 | 结果 |
| --- | --- | --- |
| **demo1** 虚拟相机+推理 | 相机色块质心 vs 桥接真值；推理契约 | ✅ **PASS**：质心误差 0.93px / 0.63px（<2.5px 阈值）；快照 `demo1_frame.png` |
| **demo2** reach 闭环 | 20 集滚动时域 rollout，成功率+Wilson CI | ✅ **PASS**：**95%（19/20）**，Wilson95=[76.4, 99.1]，平均 8.1 步（门禁 ≥70%） |
| **demo3** push+安全 | push 闭环 8 集；看门狗；安全层；接触读数 | ⚠️ **CHECK**：push 25%（2/8）；**看门狗 PASS**（推理崩溃→受控停止，漂移 0.00px）；**安全层 PASS**（999px/s→裁剪为 40、NaN→拦截） |

**结论**：部署链路（虚拟相机 → 桥接 → VLA 推理 → 速度控制 → 成功判定 → 安全层 → 看门狗）**功能全部验证正常**；reach 任务达到验收门禁；push 任务模型能力不足（见 §4）。

## 3 调试历程：闭环成功率 0% → 95%

| 版本 | reach SR | 根因与修复 |
| --- | --- | --- |
| v1（合成域 12 轮） | 0% | ①记录动作≠执行动作（裁剪后再加噪声）；②零动作尾帧占 60% → 输出塌缩；③tanh 饱和 |
| v2/v3（修复+辅助监督） | 25% → 45% | 加辅助定位头+线性输出；仍受"靠近→停车"行为外推限制 |
| v5（+仿真域微调+误差条件化） | 20%（域内） | 热启动微调方向正确，但尾帧歧义仍在 |
| v6（去掉尾帧） | 60%（域内） | 消除"停车"监督源 |
| **v7（+全量数据 587 集）** | **95%** | 数据规模到位，误差条件化动作头充分学习 |

## 4 已知限制与下一步

- **push 25%**：模型未学会专家的"错误侧绕行"策略。路径：①针对性增加 push 数据（含绕行轨迹）；②加大模型容量（dim/depth）；③双系统——错误恢复交给高层。
- demo1 的方向余弦检查对个别场景敏感（不同种子 0.48~0.98），以 demo2 的大样本成功率为准。
- 训练集 4361 帧对 3.3M 模型仍偏小；继续扩数据是最便宜的涨点（采集已脚本化，~7 分钟/300 集）。

## 5 复现命令（VM 上）

```bash
cd /home/earl/vla/code/test
nohup bash run_gazebo.sh > gzserver.log 2>&1 &        # 启动 headless 仿真
python3 calibrate.py                                   # 相机标定（期望 <2.5px）
python3 collect_gazebo_data.py --n 150 --out gz_data_r2.npz --max-minutes 24
python3 merge_npz.py merged_all.npz gz_data_big.npz gz_data_r2.npz
python3 make_notail.py merged_all.npz merged_notail.npz
python3 train_gz.py --data merged_notail.npz --out runs/gz_model_v7 \
    --init pretrained_v3/ckpt.pt --epochs 30 --lr 2.5e-4 --bs 128
bash run_demos.sh runs/gz_model_v7/ckpt.pt             # 三个 demo
```
