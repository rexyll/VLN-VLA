#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""test_p104.py —— P104 可离线执行的安全/连续性单元回归。

无需启动 Gazebo；在项目 ROS 2 Python 环境中运行：
    source /opt/ros/jazzy/setup.bash
    source vla_ws/install/setup.bash
    python3 tests/test_p104.py

此脚本只验证纯函数与运动路径分段逻辑，不替代 Gazebo 遮挡/机械臂碰撞仿真。
"""
import os
import sys
import unittest

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'vla_ws', 'src', 'vla_follow'))

from vla_follow import arm_kin as K  # noqa: E402
from vla_follow import drink_motion as M  # noqa: E402
from vla_follow import person_vision as V  # noqa: E402


class TestP104(unittest.TestCase):

    def test_rgbd_pair_requires_two_fresh_synchronised_frames(self):
        self.assertTrue(V.sensor_pair_fresh(10.00, 10.08, 10.12))
        self.assertFalse(V.sensor_pair_fresh(9.50, 10.08, 10.12))
        self.assertFalse(V.sensor_pair_fresh(10.00, 10.25, 10.26))
        self.assertFalse(V.sensor_pair_fresh(0.0, 10.08, 10.12))
        self.assertFalse(V.sensor_pair_fresh(None, 10.08, 10.12))

    def test_person_association_gate_is_bounded_and_age_monotonic(self):
        self.assertGreater(V.association_gate(1.0), V.association_gate(0.0))
        self.assertLessEqual(V.association_gate(20.0), 1.80)
        self.assertAlmostEqual(
            V.association_error((1.0, 1.0), (1.3, 1.4, 0.5)),
            0.5, places=6)
        self.assertIsNone(V.association_error((1.0, 1.0), None))

    def test_human_joint_rule_gate_rejects_missing_or_violating_sides(self):
        self.assertTrue(M.human_kinematics_ok({
            'L': {'bad': []}, 'R': {'bad': []}}))
        self.assertFalse(M.human_kinematics_ok({
            'L': {'bad': []}, 'R': {'bad': ['R5前臂扭转']}}))
        self.assertFalse(M.human_kinematics_ok({'L': {'bad': []}}))
        self.assertFalse(M.human_kinematics_ok({}))
        self.assertTrue(M.human_kinematics_ok(
            {'L': {'bad': [], 'out_line_mm': 120,
                   'droop_mm': 30, 'twist': 1.2}},
            min_out_line_mm={'L': 100},
            max_abs_droop_mm={'L': 120},
            max_abs_twist={'L': 2.2}))
        self.assertFalse(M.human_kinematics_ok(
            {'L': {'bad': [], 'out_line_mm': 70,
                   'droop_mm': 30, 'twist': 1.2}},
            min_out_line_mm={'L': 100}))

    def test_segment_aabb_guard_detects_crossing_and_clear_path(self):
        lo = (0.0, 0.0, 0.0)
        hi = (1.0, 1.0, 1.0)
        self.assertTrue(M._segment_hits_aabb(
            (-1.0, 0.5, 0.5), (2.0, 0.5, 0.5), lo, hi, radius=0.05))
        self.assertFalse(M._segment_hits_aabb(
            (-1.0, 2.0, 0.5), (2.0, 2.0, 0.5), lo, hi, radius=0.05))

    def test_oriented_palm_box_sat_detects_overlap_and_separation(self):
        I = np.eye(3)
        self.assertTrue(M._obb_hits_aabb(
            (0.0, 0.0, 0.0), I, (0.10, 0.05, 0.04),
            (0.08, -0.02, -0.02), (0.14, 0.02, 0.02), margin=0.0))
        self.assertFalse(M._obb_hits_aabb(
            (0.0, 0.0, 0.0), I, (0.10, 0.05, 0.04),
            (0.20, -0.02, -0.02), (0.24, 0.02, 0.02), margin=0.0))

        angle = np.deg2rad(45.0)
        Rz = np.array([[np.cos(angle), -np.sin(angle), 0.0],
                       [np.sin(angle), np.cos(angle), 0.0],
                       [0.0, 0.0, 1.0]])
        # Thin rotated palm: box near +y is outside its rotated narrow width.
        self.assertFalse(M._obb_hits_aabb(
            (0.0, 0.0, 0.0), Rz, (0.50, 0.05, 0.04),
            (-0.05, 0.35, -0.03), (0.05, 0.45, 0.03), margin=0.0))

    def test_hang_recovery_has_three_ordered_human_motion_stages(self):
        q0 = K.Q_REST.copy()
        q0[3] = -0.85
        q0[4:] = np.array([0.25, 0.35, -0.20])
        fold, shoulder, extend = K.hang_path_stages(q0)
        self.assertGreater(len(fold), 0)
        self.assertGreater(len(shoulder), 0)
        self.assertGreater(len(extend), 0)

        q1 = fold[-1]
        q2 = shoulder[-1]
        q3 = extend[-1]
        # Stage 1: shoulder stays fixed; elbow folds and wrist rotates to neutral.
        np.testing.assert_allclose(q1[:3], q0[:3], atol=1e-9)
        self.assertAlmostEqual(q1[3], K.HANG_FOLD, places=6)
        np.testing.assert_allclose(q1[4:], np.zeros(3), atol=1e-9)
        # Stage 2: only shoulder axes return to neutral; elbow remains folded.
        np.testing.assert_allclose(q2[:3], np.zeros(3), atol=1e-9)
        self.assertAlmostEqual(q2[3], q1[3], places=9)
        # Stage 3: extend elbow to the natural hang pose.
        np.testing.assert_allclose(q3, K.Q_HANG, atol=1e-9)

        for stage in (fold, shoulder, extend):
            previous = None
            for q in stage:
                if previous is not None:
                    self.assertLessEqual(
                        float(np.max(np.abs(q - previous))), 0.300001)
                previous = q

    def test_table_collision_guard_covers_table_top_and_legs(self):
        boxes = M._table_collision_boxes()
        names = {item[0] for item in boxes}
        self.assertIn('table_top', names)
        self.assertIn('table_lip', names)
        self.assertEqual(sum(1 for b in boxes if b[0] == 'table_leg'), 4)
        for _name, _lo, _hi in boxes:
            self.assertEqual(len(_lo), 3)
            self.assertEqual(len(_hi), 3)
            self.assertTrue(all(float(a) < float(b)
                                for a, b in zip(_lo, _hi)))


if __name__ == '__main__':
    unittest.main(verbosity=2)
