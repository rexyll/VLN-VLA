#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""diag_p103_pour.py —— 只跑"抓杯→举杯→抓壶→倒水"这一段, 逐步打印关节姿态。

用户 2026-10-28 报: 倒水时左臂肘部朝内。本脚本把每一步的实测关节角、肘点、
掌心横向位置、前臂扭转量全部打出来(口径见 `drink_motion.arm_state`), 用来
定位是哪一步落到了"肘内收"的分支上, 也用来复验修完之后的结果。

用法(世界已起):
    source /opt/ros/jazzy/setup.bash; source vla_ws/install/setup.bash
    python3 tests/diag/diag_p103_pour.py
"""
import json
import os
import sys

import numpy as np
import rclpy

from vla_follow import arm_kin as K
from vla_follow import drink_motion as M
from vla_follow import drink_poses as P

OUT = os.path.join(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))), 'vla_ws', 'logs',
    'diag_p103.json')


def main():
    from vla_follow.drink_task import DrinkTask
    rclpy.init()
    node = DrinkTask()
    node.spin_for(3.0)
    rec = {}
    T_serve = K.frame(K.rot_z(0.0), (M.L.SERVE_POSE[0], M.L.SERVE_POSE[1], 0.0))
    print('摆位: %.1fmm' % M.place_robot(node, T_serve))
    node.home()
    node.spin_for(1.0)
    rec['home'] = M.log_arm(node, '复位')

    out = M.pour_into_cup(node, fill=0.65)
    rec['pour'] = out.get('arm') or {}
    rec['res'] = out.get('res') or {}
    print('--- 关节姿态 ---')
    for tag, sides in (('举杯', rec['pour'].get('lift_cup') or {}),
                       ('倒水', rec['pour'].get('pour') or {})):
        for s, st in sides.items():
            print('%-4s %s q=%s 肘y=%+.0f 掌y=%+.0f 外扩线=%+.0f twist=%+.2f'
                  ' up_roll=%+.2f %s'
                  % (tag, s, st['q'], st['elbow_mm'], st['palm_mm'],
                     st['out_line_mm'], st['twist'], st['up_roll'],
                     ('★' + ' '.join(st['bad'])) if st['bad'] else 'OK'))
    print('--- 步骤残差(mm) ---')
    for k, v in rec['res'].items():
        print('  %-14s %s' % (k, v))
    print('ok=%s' % out.get('ok'))
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, 'w', encoding='utf-8') as f:
        json.dump(rec, f, ensure_ascii=False, indent=1, default=float)
    print('写入', OUT)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
