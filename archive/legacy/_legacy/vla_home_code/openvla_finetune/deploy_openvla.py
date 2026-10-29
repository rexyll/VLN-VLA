# -*- coding: utf-8 -*-
"""
deploy_openvla.py —— OpenVLA 真机部署参考回路（docs/06-部署与推理 的可运行骨架）

要点
    - predict_action(..., unnorm_key=...) 是官方 API：内部用模型 config 里的
      dataset_statistics 反归一化 —— 所以微调时归档统计量不是可选项（docs/04 §4）。
    - 动作 = base 坐标系 EEF delta (7,)，含夹爪维；gripper 语义（0=开还是关）
      以训练数据元数据为准，接错方向 = 现场事故（docs/09 §2）。
    - 控制频率 5Hz 起步；安全层思路与 mini_vla/robot_loop.py 相同，
      硬件急停永远独立于本程序。
用法:
    python deploy_openvla.py --unnorm-key bridge_orig --instruction "pick up the cup"
    （--unnorm-key = 微调数据集在 dataset_statistics 中的键名）
"""
from __future__ import annotations

import argparse
import time

import numpy as np
import torch

PROMPT_TEMPLATE = "In: What action should the robot take to {instruction}?\nOut:"


def get_frame(camera: str) -> np.ndarray:
    """相机帧获取。TODO(真实项目): 替换为你的相机 SDK。"""
    if camera == "demo":
        return (np.random.rand(224, 224, 3) * 255).astype(np.uint8)   # 占位帧
    # RealSense 示例（pip install pyrealsense2）:
    #   import pyrealsense2 as rs
    #   frames = pipeline.wait_for_frames()
    #   return np.asanyarray(frames.get_color_frame().get_data())
    raise NotImplementedError(f"camera {camera!r} not wired")


class RobotArm:
    """机器人接口。TODO(真实项目): 换成 ROS2 / EtherCAT / 厂商 SDK。"""

    def execute_eef_delta(self, delta7: np.ndarray, duration: float) -> None:
        print(f"[robot] delta={np.round(delta7, 3).tolist()} over {duration:.2f}s")
        # 生产实现要点：
        #   1) 目标位姿 = 当前位姿 ⊕ delta（注意旋转增量用轴角/四元数合成，别欧拉直接加）
        #   2) 交给 100Hz~1kHz 低层控制器插值跟踪，别让策略直接写关节
        #   3) 下发前过安全层（限位/限速/碰撞），docs/06 §5


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default="openvla/openvla-7b",
                    help="或 LoRA 合并后的本地权重路径")
    ap.add_argument("--unnorm-key", required=True)
    ap.add_argument("--instruction", required=True)
    ap.add_argument("--camera", default="demo")
    ap.add_argument("--hz", type=float, default=5.0)
    ap.add_argument("--max-steps", type=int, default=60)
    args = ap.parse_args()

    from PIL import Image
    from transformers import AutoModelForVision2Seq, AutoProcessor

    processor = AutoProcessor.from_pretrained(args.model, trust_remote_code=True)
    model = AutoModelForVision2Seq.from_pretrained(
        args.model, torch_dtype=torch.bfloat16, trust_remote_code=True).to("cuda")
    model.eval()

    # 提示模板必须与训练逐字符一致（docs/09 §6：模板不一致 = 语言泛化直接失效）
    prompt = PROMPT_TEMPLATE.format(instruction=args.instruction)
    robot = RobotArm()
    period = 1.0 / args.hz

    print(f"[deploy] model={args.model}  instruction={args.instruction!r}  hz={args.hz}")
    for step in range(args.max_steps):
        t0 = time.perf_counter()
        image = Image.fromarray(get_frame(args.camera))
        inputs = processor(prompt, image).to("cuda", dtype=torch.bfloat16)
        with torch.no_grad():
            action, _, _ = model.predict_action(
                **inputs, unnorm_key=args.unnorm_key, do_sample=False)
        action = np.asarray(action, dtype=np.float64)   # 未归一化 EEF delta (7,)
        robot.execute_eef_delta(action, duration=period)
        print(f"step {step:03d}  loop={(time.perf_counter()-t0)*1000:.0f}ms")
        time.sleep(max(0.0, period - (time.perf_counter() - t0)))


if __name__ == "__main__":
    main()
