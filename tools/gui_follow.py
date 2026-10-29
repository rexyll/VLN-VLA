#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""gui_follow.py —— 让 Gazebo 的 3D 视图镜头跟着某个模型走（"跟随"模式）。

为什么要这个: 看机器人实际运行时, 人/机器人一直在场地上走, 固定机位要么
框不住、要么小到看不清动作。gz-sim 的 MinimalScene 有自带的 follow 模式,
发一条 `/gui/follow`（gz.msgs.StringMsg, 内容 = 模型名）就会把相机锚到那个
模型上 —— 鼠标拖动仍可绕着它转, 滚轮缩放, 完全是"自由视角 + 镜头跟人"。

用法:
    python3 tools/gui_follow.py robot        # 跟机器人
    python3 tools/gui_follow.py person       # 跟人
    python3 tools/gui_follow.py cup          # 跟杯子（看交接/双手接杯最近）
    python3 tools/gui_follow.py off          # 取消跟随

顺序很重要（P96 实测踩过）:
    直接发 /gui/follow 之后**再**发 /gui/camera/pose 是无效的 —— 跟随模式的
    机位是"开跟随那一刻"的相机位姿, 之后发的 pose 会被它覆盖。所以必须
    ①先查到目标的世界位置  ②按它把相机摆到位  ③**最后**才开跟随。

注意:
  * 必须先有 GUI 窗口（bash tools/open_gui.sh [wide]）。
  * 人的关节不在 /drink/joint_state 里（§6.4 #60）, 看"双手接杯"动作只能靠
    这个视角凑近看 —— 真值判据在 test_p96.py 里。
"""
import math
import sys

import gz.transport13 as gz_transport
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.pose_v_pb2 import Pose_V as GzPoseV
from gz.msgs10.stringmsg_pb2 import StringMsg as GzString

TRUTH = '/world/follow_world/dynamic_pose/info'
# 相机相对被跟随目标的偏移: 后上方 1.9m、高 1.2m —— 双臂/双手/桌面都进画面
OFFSET = (-1.35, -1.45, 1.20)
PITCH = 0.36            # 低头 ~21°
YAW = math.atan2(-OFFSET[1], -OFFSET[0])   # 从偏移点回看目标


def quat_from_rpy(roll, pitch, yaw):
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def target_xyz(gz_node, name):
    """从 dynamic_pose/info 里取模型的世界位置（link 给的是相对模型的位姿）。"""
    box = {}

    def cb(msg, _b=box):
        _b['msg'] = msg

    gz_node.subscribe(GzPoseV, TRUTH, cb)
    import time
    t0 = time.time()
    while time.time() - t0 < 2.0 and 'msg' not in box:
        time.sleep(0.05)
    if 'msg' not in box:
        return None
    for p in box['msg'].pose:
        if p.name == name:
            return (p.position.x, p.position.y, p.position.z)
    return None


def main():
    target = sys.argv[1] if len(sys.argv) > 1 else 'robot'
    # 第二个参数 = 机位比例(默认 1.0 = 后上方 2.3m)。0.6 就是"凑近看手"，
    # 屏幕小/想看抓取细节时用；改动只缩放偏移，俯角 YAW/PITCH 不变。
    k = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
    off = tuple(v * k for v in OFFSET)
    gz_node = gz_transport.Node()

    if target in ('off', 'none', '0'):
        m = GzString()
        m.data = ''
        node = gz_transport.Node()
        node.request('/gui/follow', m, GzString, GzString, 2000)
        print('[gui_follow] 已取消跟随')
        return 0

    xyz = target_xyz(gz_node, target)
    if xyz is None:
        print('[gui_follow] 真值里没有 %r, 只切跟随、不动机位' % target)
    else:
        px = xyz[0] + off[0]
        py = xyz[1] + off[1]
        pz = xyz[2] + off[2]
        qx, qy, qz, qw = quat_from_rpy(0.0, PITCH, YAW)
        p = Pose()
        p.name = '3D View'
        p.position.x, p.position.y, p.position.z = px, py, pz
        p.orientation.x, p.orientation.y = qx, qy
        p.orientation.z, p.orientation.w = qz, qw
        node = gz_transport.Node()
        ok = node.request('/gui/camera/pose', p, Pose, Pose, 2000)
        if not ok:
            pub = node.advertise('/gui/camera/pose', Pose)
            for _ in range(6):
                pub.publish(p)
        print('[gui_follow] 机位 (%.2f, %.2f, %.2f)  目标 %s 在 (%.2f, %.2f)'
              % (px, py, pz, target, xyz[0], xyz[1]))

    m = GzString()
    m.data = target
    node = gz_transport.Node()
    ok = node.request('/gui/follow', m, GzString, GzString, 2000)
    if not ok:
        pub = node.advertise('/gui/follow', GzString)
        for _ in range(6):
            pub.publish(m)
    print('[gui_follow] 跟随目标 = %r  （滚轮缩放 / 左键拖动旋转）' % target)
    return 0


if __name__ == '__main__':
    sys.exit(main())
