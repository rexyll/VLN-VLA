#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""把某个模型摆到指定位姿: python3 gz_setpose.py <name> x y z roll pitch yaw"""
import math
import sys

from gz.transport13 import Node
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.boolean_pb2 import Boolean

name = sys.argv[1]
x, y, z, r, p, yw = [float(v) for v in sys.argv[2:8]]


def euler_to_quat(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


qx, qy, qz, qw = euler_to_quat(r, p, yw)
p_msg = Pose()
p_msg.name = name
p_msg.position.x = x
p_msg.position.y = y
p_msg.position.z = z
p_msg.orientation.x = qx
p_msg.orientation.y = qy
p_msg.orientation.z = qz
p_msg.orientation.w = qw

n = Node()
resp = n.request('/world/follow_world/set_pose', p_msg, Pose, Boolean, 3000)
print('SETPOSE', name, (x, y, z, r, p, yw), '->', resp)
