# VLA 代码包

两个互补的实现路径（详细讲解见 `../08-完整实现代码.md`）：

## 1. mini_vla —— 教学全流程（CPU 可跑通）

合成 2D 操作任务的完整 VLA 闭环：数据生成 → 训练 → rollout 评估 → 推理服务 → 控制回路。

| 文件 | 职责 | 对应文档 |
| --- | --- | --- |
| `env_synth.py` | 合成环境 + 脚本专家 + 数据生成 | 04 数据体系 |
| `dataset.py` | 按需渲染数据集、动作块、归一化、词表 | 04 数据体系 |
| `model.py` | MiniVLA（视觉+语言+本体 → 动作块） | 01 基本原理 |
| `train.py` | 训练循环（掩码 L1、warmup+cosine、存 best） | 05 训练流程 |
| `evaluate.py` | rollout 闭环评估 + Wilson 置信区间 | 10 评估与安全 |
| `server.py` | FastAPI 推理服务 | 06 部署与推理 |
| `robot_loop.py` | 控制回路 + 安全层 + 看门狗 | 06 部署与推理 |

## 2. openvla_finetune —— 工程路径参考（需 GPU 与官方仓库环境）

| 文件 | 职责 |
| --- | --- |
| `finetune_openvla_lora.py` | OpenVLA 7B LoRA 微调骨架（分位数归一化、动作 token 化、四件套归档） |
| `deploy_openvla.py` | 真机部署回路（官方 predict_action、5Hz、安全接入点） |

## 环境与运行

```bash
# 基础依赖
pip install "torch>=2.1" numpy

# mini_vla（CPU 即可，约几分钟）
cd mini_vla
python env_synth.py data_synth.npz 200          # 1. 生成 400 条合成演示
python train.py --data data_synth.npz --out runs/mini_vla   # 2. 训练
python evaluate.py --ckpt runs/mini_vla/ckpt.pt # 3. rollout 评估
python server.py --ckpt runs/mini_vla/ckpt.pt --port 8000   # 4. 起推理服务
python robot_loop.py --mode server --api http://127.0.0.1:8000 --task push  # 5. 控制回路
# （第 5 步也可用 --mode local 跳过服务直接进程内推理）

# server 模式额外依赖
pip install fastapi uvicorn pillow requests

# openvla 参考（需按官方仓库准备环境，显存 24GB+）
pip install transformers peft
```

> 预期结果参考：40 epoch 后 reach 任务 rollout 成功率约 85–100%，push 约 60–95%
> （合成任务 + 小模型，数字只用于验证管线正确性，不代表生产精度）。
