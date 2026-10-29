#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gui_cam.py —— 运行时把 Gazebo GUI 的 3D 视图相机推到指定机位。

为什么不用改 ~/.gz/sim/8/gui.config 的 <camera_pose>:
    实测那条对**已经跑起来的** GUI 不生效(只在启动那一刻读, 而且 GUI 正常退出
    SIGTERM 时还会把自己的当前机位写回去, 覆盖掉手工改的值; 所以改配置前必须
    pkill -9)。反复"改配置 → 重启 GUI → 截图"一轮要 15s, 调机位极慢。
    gz-sim 的 MinimalScene 订阅 `/gui/camera/pose`, 收一条 gz.msgs.Pose 立刻改
    机位 —— 即时生效、不动文件、不重启。

用法:
    python3 tools/gui_cam.py X Y Z PITCH YAW [ROLL] [NAME]
    默认 NAME = "3D View"(即 gui.config 里 MinimalScene 的插件名)

注意:
  * 角度单位是**弧度**。pitch 为**正**表示低头向下看; 写负角会朝天, 画面只剩
    背景色(在本项目里是浅灰 0.8), 极易误判成"黑屏/没渲染"。
  * 本机 gz Python 绑定只到 gz.msgs10(common5/math7/msgs10/sim8/transport13),
    没有 msgs11 —— 用 gz.msgs11 会 ModuleNotFoundError。
  * 机位语义与 gz_camera 一致: 相机放在 (X,Y,Z), 先绕 Z 转 YAW, 再本身绕 Y
    转 PITCH(正=低头), 沿自身 +X 方向看。
"""
import math
import sys

from gz.msgs10.pose_pb2 import Pose
from gz.transport13 import Node


def quat_from_rpy(roll, pitch, yaw):
    """ZYX 内旋 (yaw -> pitch -> roll) 的等价四元数 (x, y, z, w)。"""
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def main():
    if len(sys.argv) < 6:
        print(__doc__)
        return 2
    x, y, z, pitch, yaw = [float(a) for a in sys.argv[1:6]]
    roll = float(sys.argv[6]) if len(sys.argv) > 6 else 0.0
    name = sys.argv[7] if len(sys.argv) > 7 else "3D View"

    qx, qy, qz, qw = quat_from_rpy(roll, pitch, yaw)
    msg = Pose()
    msg.name = name
    msg.position.x, msg.position.y, msg.position.z = x, y, z
    msg.orientation.x, msg.orientation.y = qx, qy
    msg.orientation.z, msg.orientation.w = qz, qw

    node = Node()
    ok = node.request("/gui/camera/pose", msg, Pose, Pose, 2000)
    # /gui/camera/pose 有时是"发布/订阅"而不是"服务", 主题方式再兜一次
    if not ok:
        pub = node.advertise("/gui/camera/pose", Pose)
        for _ in range(6):
            pub.publish(msg)
        print("[gui_cam] 已发布到 /gui/camera/pose (主题方式)")
    else:
        print("[gui_cam] 已请求 /gui/camera/pose (服务方式)")
    print("[gui_cam] pos=(%.2f, %.2f, %.2f) roll=%.3f pitch=%.3f yaw=%.3f name=%s"
          % (x, y, z, roll, pitch, yaw, name))
    return 0


if __name__ == "__main__":
    sys.exit(main())
