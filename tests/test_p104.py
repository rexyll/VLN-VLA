#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""test_p104.py —— P104 可离线执行的安全/连续性单元回归。

无需启动 Gazebo；在项目 ROS 2 Python 环境中运行：
    source /opt/ros/jazzy/setup.bash
    source vla_ws/install/setup.bash
    python3 tests/test_p104.py

此脚本只验证纯函数与运动路径分段逻辑，不替代 Gazebo 遮挡/机械臂碰撞仿真。
"""
import json
import os
import sys
import threading
import tempfile
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET
from drink_evidence import (REQUIRED_SHOT_LABELS, missing_required_shots,
                            should_stop_terminal_observer,
                            summarize_round_evidence,
                            validate_observer_result,
                            archive_observer_screenshots,
                            write_json_atomic, summarize_numeric_values,
                            finite_numeric_value)

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'vla_ws', 'src', 'vla_follow'))


def _stub_module(name, package=False):
    """Install a minimal import placeholder for an unused runtime dependency."""
    import types
    module = types.ModuleType(name)
    if package:
        module.__path__ = []
    sys.modules[name] = module
    return module


def _stub_ros_gz_imports_for_pure_tests():
    """P104 tests exercise pure helpers; they never instantiate ROS/Gazebo nodes."""
    try:
        from rclpy.node import Node  # noqa: F401
    except ModuleNotFoundError as exc:
        if exc.name not in ('rclpy', 'rclpy.node'):
            raise
        rclpy = _stub_module('rclpy', package=True)
        node_mod = _stub_module('rclpy.node')
        node_mod.Node = type('Node', (), {})
        rclpy.node = node_mod

    try:
        from std_msgs.msg import String  # noqa: F401
    except ModuleNotFoundError as exc:
        if exc.name not in ('std_msgs', 'std_msgs.msg'):
            raise
        std_msgs = _stub_module('std_msgs', package=True)
        msg_mod = _stub_module('std_msgs.msg')
        msg_mod.String = type('String', (), {})
        std_msgs.msg = msg_mod

    try:
        import gz.transport13  # noqa: F401
    except ModuleNotFoundError as exc:
        if exc.name not in ('gz', 'gz.transport13'):
            raise
        gz = _stub_module('gz', package=True)
        transport_mod = _stub_module('gz.transport13')
        gz.transport13 = transport_mod

    try:
        from gz.msgs10.image_pb2 import Image  # noqa: F401
    except ModuleNotFoundError as exc:
        if exc.name not in ('gz', 'gz.msgs10', 'gz.msgs10.image_pb2'):
            raise
        gz = sys.modules.get('gz') or _stub_module('gz', package=True)
        msgs = sys.modules.get('gz.msgs10') or _stub_module(
            'gz.msgs10', package=True)
        image_mod = _stub_module('gz.msgs10.image_pb2')
        image_mod.Image = type('Image', (), {})
        gz.msgs10 = msgs
        msgs.image_pb2 = image_mod

    try:
        from gz.msgs10.pose_v_pb2 import Pose_V  # noqa: F401
    except ModuleNotFoundError as exc:
        if exc.name not in ('gz', 'gz.msgs10', 'gz.msgs10.pose_v_pb2'):
            raise
        gz = sys.modules.get('gz') or _stub_module('gz', package=True)
        msgs = sys.modules.get('gz.msgs10') or _stub_module(
            'gz.msgs10', package=True)
        pose_mod = _stub_module('gz.msgs10.pose_v_pb2')
        pose_mod.Pose_V = type('Pose_V', (), {})
        gz.msgs10 = msgs
        msgs.pose_v_pb2 = pose_mod


_stub_ros_gz_imports_for_pure_tests()

from vla_follow import arm_kin as K  # noqa: E402
from vla_follow import drink_motion as M  # noqa: E402
from vla_follow import person_vision as V  # noqa: E402
from vla_follow import carry as C  # noqa: E402


class TestP104(unittest.TestCase):

    def test_drink_summary_metrics_exclude_boolean_and_non_finite_values(self):
        summary = summarize_numeric_values([
            1, 3.5, True, False, float('nan'), float('inf'),
            float('-inf'), '4', None,
        ])
        self.assertEqual(summary, {
            'median': 2.25, 'min': 1.0, 'max': 3.5,
        })
        self.assertIsNone(summarize_numeric_values([
            True, False, float('nan'), float('inf'), 'not-a-number',
        ]))
        self.assertIsNone(summarize_numeric_values([10 ** 10000]))

    def test_numeric_summary_normalizes_values_and_avoids_median_overflow(self):
        largest = sys.float_info.max
        self.assertEqual(finite_numeric_value(2), 2.0)
        self.assertEqual(finite_numeric_value(-0.5), -0.5)
        for invalid in (True, False, float('nan'), float('inf'),
                        float('-inf'), '3', None, 10 ** 10000):
            self.assertIsNone(finite_numeric_value(invalid))

        same_sign = summarize_numeric_values([largest, largest])
        self.assertEqual(same_sign['median'], largest)
        self.assertEqual(same_sign['min'], largest)
        self.assertEqual(same_sign['max'], largest)
        opposite_sign = summarize_numeric_values([-largest, largest])
        self.assertEqual(opposite_sign['median'], 0.0)

    def test_json_publication_rejects_nonstandard_numeric_constants(self):
        with tempfile.TemporaryDirectory() as root:
            target = os.path.join(root, 'summary.json')
            with open(target, 'w', encoding='utf-8') as stream:
                stream.write('previous-summary')

            with self.assertRaises(ValueError):
                write_json_atomic({'nested': {'metric': float('nan')}}, target)

            with open(target, encoding='utf-8') as stream:
                self.assertEqual(stream.read(), 'previous-summary')
            leftovers = [name for name in os.listdir(root)
                         if name.startswith('.drink-json-')]
            self.assertEqual(leftovers, [])

    def test_repeat_runner_rechecks_round_acceptance_before_next_round(self):
        runner_path = os.path.join(ROOT, 'tests', 'drink_repeat.sh')
        with open(runner_path, encoding='utf-8') as stream:
            runner = stream.read()
        gate = runner.index('summary = summarize_round_evidence(result, root)')
        stop = runner.index('FAILED=1', gate)
        cooldown = runner.index('if (( i < N && COOLDOWN > 0 ))')
        self.assertLess(gate, stop)
        self.assertLess(stop, cooldown)

    def test_repeat_summary_requires_runner_exit_success(self):
        runner_path = os.path.join(ROOT, 'tests', 'drink_repeat.sh')
        with open(runner_path, encoding='utf-8') as stream:
            runner = stream.read()
        self.assertIn('root, failed, *paths = sys.argv[1:]', runner)
        self.assertIn('"all_passed": (int(failed) == 0 and len(rows) == int(requested)', runner)

    def test_drink_screenshot_acceptance_requires_each_unique_event(self):
        complete = [
            'arrive', 'present', 'grip', 'drink', 'takeback', 'placed',
            'handback',
        ]
        self.assertEqual(REQUIRED_SHOT_LABELS, set(complete))
        self.assertEqual(missing_required_shots(complete), [])
        self.assertEqual(
            missing_required_shots(complete + ['present', 'present']), [])
        self.assertEqual(
            missing_required_shots(['present', 'present', 'drink']),
            ['arrive', 'grip', 'handback', 'placed', 'takeback'])
        self.assertEqual(
            missing_required_shots(None), sorted(REQUIRED_SHOT_LABELS))

    def test_observer_result_contract_allows_optional_evidence(self):
        self.assertEqual(validate_observer_result({'ok': True}), [])
        self.assertEqual(validate_observer_result({
            'shots': ['one.png'], 'evidence': {},
        }), ['one.png'])
        self.assertEqual(validate_observer_result({
            'shots': [], 'evidence': {'source': 'observer'},
        }), [])

        invalid_results = (
            None,
            [],
            {'shots': None},
            {'shots': 'one.png'},
            {'shots': [None]},
            {'shots': ['']},
            {'evidence': None},
            {'evidence': []},
            {'evidence': 'metadata'},
        )
        for result in invalid_results:
            with self.subTest(result=result):
                with self.assertRaises(ValueError):
                    validate_observer_result(result)

    def test_json_publication_is_atomic_and_cleans_failed_temporary_file(self):
        with tempfile.TemporaryDirectory() as root:
            target = os.path.join(root, 'summary.json')
            with open(target, 'w', encoding='utf-8') as stream:
                stream.write('previous-summary')

            with patch('os.replace', side_effect=OSError('simulated publish failure')):
                with self.assertRaises(OSError):
                    write_json_atomic({'all_passed': True}, target)

            with open(target, encoding='utf-8') as stream:
                self.assertEqual(stream.read(), 'previous-summary')
            leftovers = [name for name in os.listdir(root)
                         if name.startswith('.drink-json-')]
            self.assertEqual(leftovers, [])

            write_json_atomic({'all_passed': True}, target)
            with open(target, encoding='utf-8') as stream:
                self.assertEqual(json.load(stream), {'all_passed': True})

            # If fdopen fails before taking ownership, close the raw descriptor.
            with patch('os.fdopen', side_effect=OSError('simulated fdopen failure')):
                with patch('os.close', wraps=os.close) as close_fd:
                    with self.assertRaises(OSError):
                        write_json_atomic({'all_passed': False}, target)
                    close_fd.assert_called_once()
            with open(target, encoding='utf-8') as stream:
                self.assertEqual(json.load(stream), {'all_passed': True})
            leftovers = [name for name in os.listdir(root)
                         if name.startswith('.drink-json-')]
            self.assertEqual(leftovers, [])

            # Serialization failures after opening the temporary file must
            # close it, remove the temporary path, and preserve old SUMMARY.
            with self.assertRaises(TypeError):
                write_json_atomic({'not_json': object()}, target)
            with open(target, encoding='utf-8') as stream:
                self.assertEqual(json.load(stream), {'all_passed': True})
            leftovers = [name for name in os.listdir(root)
                         if name.startswith('.drink-json-')]
            self.assertEqual(leftovers, [])

    def test_screenshot_archive_is_atomic_and_adds_optional_evidence(self):
        with tempfile.TemporaryDirectory() as root:
            source_dir = os.path.join(root, 'source')
            os.makedirs(source_dir)
            sources = []
            for index in range(2):
                path = os.path.join(source_dir, '%d.png' % index)
                with open(path, 'wb') as stream:
                    stream.write(b'image-%d' % index)
                sources.append(path)

            result = {'ok': True, 'shots': sources}
            target = os.path.join(root, 'archive', 'round_shots')
            archived = archive_observer_screenshots(result, root, target)
            self.assertEqual(archived['evidence']['screenshots_archived'], 2)
            self.assertEqual(len(archived['shots']), 2)
            self.assertTrue(all(os.path.isfile(os.path.join(root, path))
                                for path in archived['shots']))
            with self.assertRaises(FileExistsError):
                archive_observer_screenshots(result, root, target)

            failed_target = os.path.join(root, 'archive', 'failed_shots')
            failed_result = {'shots': [sources[0],
                                       os.path.join(source_dir, 'missing.png')]}
            with self.assertRaises(FileNotFoundError):
                archive_observer_screenshots(
                    failed_result, root, failed_target)
            self.assertFalse(os.path.lexists(failed_target))
            leftovers = [name for name in os.listdir(os.path.dirname(failed_target))
                         if name.startswith('.drink-shots-')]
            self.assertEqual(leftovers, [])

            outside = os.path.join(os.path.dirname(root), 'private-evidence.png')
            with open(outside, 'wb') as stream:
                stream.write(b'outside')
            with self.assertRaises(ValueError):
                archive_observer_screenshots(
                    {'shots': [outside]}, root,
                    os.path.join(root, 'archive', 'outside_shots'))
            with self.assertRaises(ValueError):
                archive_observer_screenshots(
                    {'shots': []}, root,
                    os.path.join(os.path.dirname(root), 'outside_archive'))

            # A lexically in-repository target must still be rejected when
            # an existing parent symlink resolves outside the repository.
            archive_link = os.path.join(root, 'archive_link')
            os.symlink(os.path.dirname(root), archive_link)
            with self.assertRaises(ValueError):
                archive_observer_screenshots(
                    {'shots': []}, root,
                    os.path.join(archive_link, 'escaped_archive'))

    def test_round_summary_requires_complete_archived_screenshots(self):
        with tempfile.TemporaryDirectory() as root:
            shot_dir = os.path.join(root, 'round_shots')
            os.makedirs(shot_dir)
            shots = []
            for index, label in enumerate(sorted(REQUIRED_SHOT_LABELS)):
                path = os.path.join(shot_dir, '%02d_%s.png' % (index, label))
                with open(path, 'wb') as stream:
                    stream.write(b'png-placeholder')
                shots.append(os.path.relpath(path, root))
            labels = sorted(REQUIRED_SHOT_LABELS)
            result = {
                'ok': True,
                'shots': shots,
                'shot_labels': labels,
                'evidence': {'screenshots_archived': len(shots)},
            }
            summary = summarize_round_evidence(result, root)
            self.assertTrue(summary['ok'])
            self.assertTrue(summary['archive_ok'])
            self.assertEqual(summary['screenshots_archived'], 7)

            missing_file = dict(result, shots=shots[:-1] + ['round_shots/missing.png'])
            self.assertFalse(summarize_round_evidence(
                missing_file, root)['archive_ok'])

            wrong_count = dict(result, evidence={'screenshots_archived': 6})
            self.assertFalse(summarize_round_evidence(
                wrong_count, root)['archive_ok'])

            missing_label = dict(result, shot_labels=labels[:-1])
            self.assertFalse(summarize_round_evidence(
                missing_label, root)['archive_ok'])

            duplicate_label = dict(result, shot_labels=labels + [labels[0]])
            self.assertFalse(summarize_round_evidence(
                duplicate_label, root)['archive_ok'])

            duplicate_path = dict(result, shots=shots[:-1] + [shots[0]])
            self.assertFalse(summarize_round_evidence(
                duplicate_path, root)['archive_ok'])

            outside = os.path.abspath(os.path.join(root, '..', 'outside.png'))
            with open(outside, 'wb') as stream:
                stream.write(b'outside')
            outside_path = dict(result, shots=shots[:-1] + [outside])
            self.assertFalse(summarize_round_evidence(
                outside_path, root)['archive_ok'])

            malformed_labels = dict(result, shot_labels=[{}])
            self.assertFalse(summarize_round_evidence(
                malformed_labels, root)['archive_ok'])

            nul_path = dict(result, shots=shots[:-1] + ['round_shots/bad\x00.png'])
            self.assertFalse(summarize_round_evidence(
                nul_path, root)['archive_ok'])

            malformed_result = summarize_round_evidence(None, root)
            self.assertFalse(malformed_result['ok'])

            malformed_fields = dict(result, shots=None, evidence=[])
            self.assertFalse(summarize_round_evidence(
                malformed_fields, root)['archive_ok'])

            task_failed = dict(result, ok=False)
            self.assertFalse(summarize_round_evidence(
                task_failed, root)['ok'])

            truthy_string = dict(result, ok="false")
            self.assertFalse(summarize_round_evidence(
                truthy_string, root)['ok'])

    def test_done_state_waits_for_final_placed_screenshot(self):
        self.assertFalse(should_stop_terminal_observer(
            'DONE', [], 0.0))
        self.assertTrue(should_stop_terminal_observer(
            'DONE', ['placed'], 0.0))
        self.assertTrue(should_stop_terminal_observer(
            'DONE', [], 3.0))
        self.assertTrue(should_stop_terminal_observer(
            'ERROR', [], 0.0))
        self.assertFalse(should_stop_terminal_observer(
            'PRESENT', [], 10.0))

    def test_rgbd_pair_requires_two_fresh_synchronised_frames(self):
        self.assertTrue(V.sensor_pair_fresh(10.00, 10.08, 10.12))
        self.assertFalse(V.sensor_pair_fresh(9.50, 10.08, 10.12))
        self.assertFalse(V.sensor_pair_fresh(10.00, 10.25, 10.26))
        self.assertFalse(V.sensor_pair_fresh(0.0, 10.08, 10.12))
        self.assertFalse(V.sensor_pair_fresh(None, 10.08, 10.12))
        self.assertTrue(V.timestamp_fresh(10.0, 10.12))
        self.assertFalse(V.timestamp_fresh(9.5, 10.12))
        self.assertFalse(V.timestamp_fresh(10.2, 10.12))
        self.assertFalse(V.timestamp_fresh(None, 10.12))
        self.assertTrue(V.sensor_pose_pair_fresh(
            10.00, 10.04, 10.06, 10.10))
        self.assertFalse(V.sensor_pose_pair_fresh(
            10.00, 10.04, 9.80, 10.10))
        self.assertFalse(V.sensor_pose_pair_fresh(
            10.00, 10.04, 10.30, 10.31))
        self.assertFalse(V.sensor_pose_pair_fresh(
            9.50, 10.04, 10.06, 10.10))

    def test_person_occlusion_prediction_is_speed_capped_and_expires(self):
        vision = object.__new__(V.PersonVision)
        vision.hist = [
            {'t': 9.0, 'wx': 0.0, 'wy': 0.0},
            {'t': 10.0, 'wx': 1.2, 'wy': 0.0},
        ]
        predicted = vision._predict_world(11.0)
        self.assertIsNotNone(predicted)
        # Raw speed is 1.2 m/s; association prediction must cap it at 0.85 m/s.
        self.assertAlmostEqual(predicted[0], 2.05, places=6)
        self.assertAlmostEqual(predicted[1], 0.0, places=6)
        self.assertAlmostEqual(predicted[2], 1.0, places=6)
        # Once the occlusion exceeds the association window, reacquisition is
        # unanchored instead of forcing the tracker to follow a stale position.
        self.assertIsNone(vision._predict_world(
            10.0 + V.TRACK_ASSOC_MAX_AGE_S + 0.01))

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
        # 单臂节点可只要求当前操作侧；双臂倒水必须显式要求 L 和 R 都有实测姿态。
        self.assertTrue(M.human_kinematics_ok(
            {'L': {'bad': []}}, required_sides=('L',)))
        self.assertFalse(M.human_kinematics_ok(
            {'L': {'bad': []}}, required_sides=('L', 'R')))
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

    def test_oriented_palm_box_sat_and_finger_geometry(self):
        lo = (0.0, 0.0, 0.0)
        hi = (1.0, 1.0, 1.0)
        identity = np.eye(3)
        self.assertTrue(M._obb_hits_aabb(
            (0.5, 0.5, 0.5), identity, (0.1, 0.1, 0.1),
            lo, hi, margin=0.0))
        # 旋转后的细长掌部 AABB 投影有重叠，但真实 OBB 与箱体分离；
        # SAT 必须识别分离轴，不能退化为简单包围盒误报。
        rot45 = K.rot((0.0, 0.0, 1.0), np.pi / 4.0)
        self.assertFalse(M._obb_hits_aabb(
            (1.6, 1.6, 0.5), rot45, (0.5, 0.05, 0.05),
            lo, hi, margin=0.0))

        # 复现 3D yaw/pitch/roll 同时存在时的 SAT 轴矩阵方向问题：
        # 旧实现会错误报告相交；修正后发现分离轴并返回 False。
        rot3d = (K.rot((0.0, 0.0, 1.0), 0.7)
                 @ K.rot((0.0, 1.0, 0.0), -0.8)
                 @ K.rot((1.0, 0.0, 0.0), 0.6))
        self.assertFalse(M._obb_hits_aabb(
            (-0.2005222, 0.6672574, 0.45492798), rot3d,
            (0.27158509, 0.0326054, 0.10873528),
            lo, hi, margin=0.0))

        segments, palm = M._arm_geometry_world('L', K.Q_HANG, (0.0, 0.0, 0.0))
        segment_names = {name for name, *_ in segments}
        self.assertIn('upper_arm', segment_names)
        self.assertIn('forearm', segment_names)
        self.assertIn('thumb', segment_names)
        self.assertEqual(sum(1 for name in segment_names
                             if name.startswith('finger_')), 4)
        center, rotation, half = palm
        self.assertEqual(center.shape, (3,))
        self.assertEqual(rotation.shape, (3, 3))
        np.testing.assert_allclose(half, 0.5 * np.asarray(K.PALM_SIZE))

    def test_person_hand_gap_gate_checks_realized_open_and_closed_width(self):
        self.assertAlmostEqual(
            M.person_hand_gap_expected_mm(0.42), 345.0, places=6)
        self.assertAlmostEqual(
            M.person_hand_gap_expected_mm(0.035), 184.0, places=6)
        self.assertTrue(M.person_hand_gap_ok(350.0, 0.42))
        self.assertFalse(M.person_hand_gap_ok(184.0, 0.42))
        self.assertTrue(M.person_hand_gap_ok(190.0, 0.035))
        self.assertFalse(M.person_hand_gap_ok(345.0, 0.035))
        self.assertFalse(M.person_hand_gap_ok(None, 0.42))
        self.assertFalse(M.person_hand_gap_ok(np.nan, 0.42))

    def test_base_pose_confirmation_rejects_nonfinite_or_out_of_tolerance(self):
        self.assertTrue(M.base_pose_converged(2.0, 0.01, 8.0))
        self.assertFalse(M.base_pose_converged(8.0, 0.01, 8.0))
        self.assertFalse(M.base_pose_converged(2.0, np.nan, 8.0))
        self.assertFalse(M.base_pose_converged(np.inf, 0.01, 8.0))
        self.assertFalse(M.base_pose_converged(None, 0.01, 8.0))

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

    def test_carry_request_ids_are_fresh_across_bus_restarts(self):
        class Node:
            def create_publisher(self, *_args):
                return object()
            def create_subscription(self, *_args):
                return object()

        seeds = iter((1700000000000000000, 1700000000000000001))
        original_time_ns = C.time.time_ns
        C.time.time_ns = lambda: next(seeds)
        try:
            first = C.CarryBus(Node())
            second = C.CarryBus(Node())
        finally:
            C.time.time_ns = original_time_ns

        self.assertEqual(first.request_seq, 1700000000000000000)
        self.assertEqual(second.request_seq, 1700000000000000001)
        self.assertLess(second.request_seq, 2 ** 64 - 1)

    def test_carry_ack_requires_exact_operation_model_and_request_id(self):
        class Logger:
            def info(self, *_args):
                pass
        class Node:
            def __init__(self):
                self.bus = None
                self.calls = 0
            def get_logger(self):
                return Logger()
            def spin_for(self, _duration):
                self.calls += 1
                if self.calls == 1:
                    self.bus._on_ack(type('Msg', (), {'data': 'ok release cup 41'})())
                elif self.calls == 2:
                    self.bus._on_ack(type('Msg', (), {'data': 'ok release all 7'})())
        node = Node()
        bus = object.__new__(C.CarryBus)
        bus.node = node
        bus.request_seq = 6
        bus.ack = None
        bus.acks = []
        bus.ack_t = 0.0
        bus.pending_requests = set()
        bus.ack_by_request = {}
        bus._ack_lock = threading.RLock()
        node.bus = bus
        emitted = []
        ok = bus._await(
            lambda rid: emitted.append(rid),
            lambda ack, rid: ack == 'ok release all %d' % rid,
            0.5)
        self.assertTrue(ok)
        self.assertEqual(emitted, [7, 7, 7, 7, 7])
        self.assertEqual(bus.acks[-1], 'ok release all 7')

    def test_carry_ack_waiters_are_isolated_during_nested_spin(self):
        class Logger:
            def info(self, *_args):
                pass
        class Node:
            def __init__(self):
                self.bus = None
                self.calls = 0
                self.outer_ids = []
                self.inner_ids = []
                self.nested_result = None
            def get_logger(self):
                return Logger()
            def spin_for(self, _duration):
                self.calls += 1
                if self.calls == 1:
                    self.nested_result = self.bus._await(
                        lambda rid: self.inner_ids.append(rid),
                        lambda ack, rid: ack == 'ok joint cup water_joint %d' % rid,
                        0.5)
                elif self.calls == 2:
                    outer_id = self.outer_ids[0]
                    inner_id = self.inner_ids[0]
                    for data in (
                            'ok release all %d' % outer_id,
                            'ok joint cup water_joint %d' % inner_id):
                        self.bus._on_ack(type('Msg', (), {'data': data})())
        node = Node()
        bus = object.__new__(C.CarryBus)
        bus.node = node
        bus.request_seq = 20
        bus.ack = None
        bus.acks = []
        bus.ack_t = 0.0
        bus.pending_requests = set()
        bus.ack_by_request = {}
        bus._ack_lock = threading.RLock()
        node.bus = bus

        outer_ok = bus._await(
            lambda rid: node.outer_ids.append(rid),
            lambda ack, rid: ack == 'ok release all %d' % rid,
            0.5)
        self.assertTrue(node.nested_result)
        self.assertTrue(outer_ok)
        self.assertEqual(node.outer_ids, [21, 21, 21, 21])
        self.assertEqual(node.inner_ids, [22, 22, 22, 22])
        self.assertEqual(bus.pending_requests, set())
        self.assertEqual(bus.ack_by_request, {})

    def test_carry_delayed_ack_ignores_stale_and_post_timeout_ids(self):
        class Logger:
            def info(self, *_args):
                pass
            def error(self, *_args):
                pass

        class Node:
            def __init__(self):
                self.bus = None
                self.calls = 0
            def get_logger(self):
                return Logger()
            def spin_for(self, _duration):
                self.calls += 1
                request_id = self.bus.request_seq
                if self.calls == 1:
                    # A delayed ACK from an older transaction must not satisfy
                    # the current waiter even when the operation is similar.
                    self.bus._on_ack(type(
                        'Msg', (), {'data': 'ok joint cup water_joint %d'
                                   % (request_id - 1)})())
                elif self.calls == 2:
                    self.bus._on_ack(type(
                        'Msg', (), {'data': 'ok joint cup water_joint %d'
                                   % request_id})())

        node = Node()
        bus = object.__new__(C.CarryBus)
        bus.node = node
        bus.request_seq = 400
        bus.ack = None
        bus.acks = []
        bus.ack_t = 0.0
        bus.pending_requests = set()
        bus.ack_by_request = {}
        bus._ack_lock = threading.RLock()
        node.bus = bus
        emitted = []
        ok = bus._await(
            lambda rid: emitted.append(rid),
            lambda ack, rid: ack == 'ok joint cup water_joint %d' % rid,
            1.0)
        self.assertTrue(ok)
        self.assertEqual(node.calls, 5)  # 2 polls + 3 confirmed retransmits
        self.assertEqual(emitted, [401, 401, 401, 401, 401])
        self.assertEqual(bus.pending_requests, set())
        self.assertEqual(bus.ack_by_request, {})

        # A duplicate ACK arriving after the waiter has completed is ignored.
        bus._on_ack(type(
            'Msg', (), {'data': 'ok joint cup water_joint 401'})())
        self.assertNotIn(401, bus.ack_by_request)

    def test_carry_ack_first_terminal_result_wins_over_duplicates_and_conflicts(self):
        class Node:
            pass

        bus = object.__new__(C.CarryBus)
        bus.node = Node()
        bus.ack = None
        bus.acks = []
        bus.ack_t = 0.0
        bus.pending_requests = {8101, 8102}
        bus.ack_by_request = {}
        bus._ack_lock = threading.RLock()

        def deliver(data):
            bus._on_ack(type('Msg', (), {'data': data})())

        # Identical duplicate is idempotent; a later conflicting ACK cannot
        # turn a terminal success into failure.
        success = 'ok joint cup water_joint 8101'
        deliver(success)
        deliver(success)
        deliver('fail joint cup water_joint 8101 queue-full')
        self.assertEqual(bus.ack_by_request[8101], success)

        # The reverse order is also deterministic: terminal queue-full reject
        # remains authoritative even if a contradictory success arrives later.
        rejected = 'fail joint cup water_joint 8102 queue-full'
        deliver(rejected)
        deliver(rejected)
        deliver('ok joint cup water_joint 8102')
        self.assertEqual(bus.ack_by_request[8102], rejected)

    def test_carry_ack_arriving_after_timeout_is_quarantined(self):
        class Logger:
            def info(self, *_args):
                pass
            def error(self, *_args):
                pass
        class Node:
            def get_logger(self):
                return Logger()
            def spin_for(self, _duration):
                raise AssertionError("zero-timeout waiter must not spin")

        bus = object.__new__(C.CarryBus)
        bus.node = Node()
        bus.request_seq = 8000
        bus.ack = None
        bus.acks = []
        bus.ack_t = 0.0
        bus.pending_requests = set()
        bus.ack_by_request = {}
        bus._ack_lock = threading.RLock()

        emitted = []
        ok = bus._await(
            lambda rid: emitted.append(rid),
            lambda ack, rid: ack == 'ok joint cup water_joint %d' % rid,
            0.0)
        self.assertFalse(ok)
        self.assertEqual(emitted, [])
        expired_request_id = 8001
        self.assertEqual(bus.pending_requests, set())

        # ACK after timeout must not be cached as a future completion.
        bus._on_ack(type(
            'Msg', (), {
                'data': 'ok joint cup water_joint %d' % expired_request_id
            })())
        self.assertNotIn(expired_request_id, bus.ack_by_request)

        # Even when a later transaction is pending, the expired ACK stays stale.
        active_request_id = 8002
        bus.pending_requests.add(active_request_id)
        bus._on_ack(type(
            'Msg', (), {
                'data': 'ok joint cup water_joint %d' % expired_request_id
            })())
        self.assertNotIn(active_request_id, bus.ack_by_request)
        self.assertNotIn(expired_request_id, bus.ack_by_request)

    def test_carry_ack_callback_racing_timeout_cleanup_leaves_no_stale_state(self):
        class Node:
            def create_publisher(self, *_args):
                return object()
            def create_subscription(self, *_args):
                return object()

        # Exercise both lock acquisition orders repeatedly. The callback either
        # publishes before cleanup (and cleanup removes it), or observes that
        # the request is no longer pending. No partial/stale state is allowed.
        bus = C.CarryBus(Node())
        for request_id in range(8100, 8200):
            with bus._ack_lock:
                bus.pending_requests.add(request_id)
            gate = threading.Barrier(3)

            def deliver_ack():
                gate.wait()
                bus._on_ack(type(
                    'Msg', (), {
                        'data': 'ok joint cup water_joint %d' % request_id
                    })())

            def timeout_cleanup():
                gate.wait()
                with bus._ack_lock:
                    bus.pending_requests.discard(request_id)
                    bus.ack_by_request.pop(request_id, None)

            callback_thread = threading.Thread(target=deliver_ack)
            cleanup_thread = threading.Thread(target=timeout_cleanup)
            callback_thread.start()
            cleanup_thread.start()
            gate.wait()
            callback_thread.join(timeout=2.0)
            cleanup_thread.join(timeout=2.0)
            self.assertFalse(callback_thread.is_alive())
            self.assertFalse(cleanup_thread.is_alive())
            with bus._ack_lock:
                self.assertNotIn(request_id, bus.pending_requests)
                self.assertNotIn(request_id, bus.ack_by_request)

    def test_carry_late_ack_cannot_complete_a_different_pending_request(self):
        class Node:
            def create_publisher(self, *_args):
                return object()
            def create_subscription(self, *_args):
                return object()

        bus = C.CarryBus(Node())
        expired_request_id = 7001
        active_request_id = 7002
        bus.pending_requests.add(active_request_id)

        # A delayed success for the expired transaction arrives while a newer
        # transaction is pending. It must remain diagnostic-only.
        bus._on_ack(type(
            'Msg', (), {
                'data': 'ok joint cup water_joint %d' % expired_request_id
            })())
        self.assertNotIn(expired_request_id, bus.ack_by_request)
        self.assertNotIn(active_request_id, bus.ack_by_request)

        # Only the ACK carrying the currently pending identity may complete it.
        expected = 'ok joint cup water_joint %d' % active_request_id
        bus._on_ack(type('Msg', (), {'data': expected})())
        self.assertEqual(bus.ack_by_request.get(active_request_id), expected)

    def test_carry_ack_callback_is_safe_under_concurrent_executor_threads(self):
        class Node:
            def create_publisher(self, *_args):
                return object()
            def create_subscription(self, *_args):
                return object()

        bus = C.CarryBus(Node())
        request_ids = list(range(1001, 1025))
        bus.pending_requests.update(request_ids)
        messages = [
            'ok joint cup water_joint %d' % request_id
            for request_id in request_ids
        ]
        threads = [
            threading.Thread(
                target=bus._on_ack,
                args=(type('Msg', (), {'data': message})(),))
            for message in messages
        ]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join(timeout=2.0)
            self.assertFalse(thread.is_alive())

        self.assertEqual(set(bus.ack_by_request), set(request_ids))
        for request_id in request_ids:
            self.assertEqual(
                bus.ack_by_request[request_id],
                'ok joint cup water_joint %d' % request_id)

    def test_carry_ack_matching_rejection_fails_without_retrying_to_timeout(self):
        class Logger:
            def error(self, *_args):
                pass
        class Node:
            def __init__(self):
                self.bus = None
                self.calls = 0
            def get_logger(self):
                return Logger()
            def spin_for(self, _duration):
                self.calls += 1
                self.bus._on_ack(type(
                    'Msg', (), {'data': 'fail joint cup water_joint 31 queue-full'})())
        node = Node()
        bus = object.__new__(C.CarryBus)
        bus.node = node
        bus.request_seq = 30
        bus.ack = None
        bus.acks = []
        bus.ack_t = 0.0
        bus.pending_requests = set()
        bus.ack_by_request = {}
        bus._ack_lock = threading.RLock()
        node.bus = bus
        emitted = []
        ok = bus._await(
            lambda rid: emitted.append(rid),
            lambda ack, rid: ack == 'ok joint cup water_joint %d' % rid,
            0.5)
        self.assertFalse(ok)
        self.assertEqual(emitted, [31])
        self.assertEqual(node.calls, 1)
        self.assertEqual(bus.pending_requests, set())

    def test_release_ack_loss_quarantines_target_until_explicit_retry(self):
        class Logger:
            def info(self, *_args):
                pass
            def warn(self, *_args):
                pass

        class Node:
            def get_logger(self):
                return Logger()

        bus = object.__new__(C.CarryBus)
        bus.node = Node()
        bus.items = {
            'cup': {'side': 'L', 'offset': np.eye(4)},
        }
        bus._items_lock = threading.RLock()
        sent = []
        bus._send_pose = lambda *args: sent.append(args)
        bus.palm_world = lambda _side: np.eye(4)

        def lose_release_ack(*_args, **_kwargs):
            # Simulate a concurrent 50Hz update while release is awaiting ACK.
            bus.update()
            return False
        bus._await = lose_release_ack
        self.assertFalse(bus.release('cup'))
        self.assertIn('cup', bus.items)
        self.assertTrue(bus.items['cup']['release_unknown'])
        self.assertNotIn('release_pending', bus.items['cup'])
        self.assertEqual(sent, [])  # no set during release or after unknown outcome

        # A later explicit release retry with a matching ACK clears the quarantine.
        bus._await = lambda *_args, **_kwargs: True
        self.assertTrue(bus.release('cup'))
        self.assertNotIn('cup', bus.items)

        # release-all ACK loss quarantines every remaining attached object.
        bus.items['kettle'] = {'side': 'R', 'offset': np.eye(4)}
        bus._await = lambda *_args, **_kwargs: False
        self.assertFalse(bus.release())
        self.assertTrue(bus.items['kettle']['release_unknown'])
        bus.update()
        self.assertEqual(sent, [])

    def test_carry_hand_to_switches_host_without_pose_discontinuity(self):
        class Logger:
            def info(self, *_args):
                pass
            def error(self, *_args):
                pass

        class Node:
            def get_logger(self):
                return Logger()

        bus = object.__new__(C.CarryBus)
        bus.node = Node()
        bus.items = {}
        bus._items_lock = threading.RLock()
        host_pose = K.trans((0.4, -0.2, 0.8))
        object_pose = K.trans((0.55, -0.1, 0.9))
        bus.host_world = lambda *_args: host_pose.copy()
        bus.obj_world = lambda _model: object_pose.copy()

        self.assertTrue(bus.hand_to(
            'cup', 'person_arms', 'person_arms_link', (0.0, 0.0, 0.1)))
        offset = bus.items['cup']['offset'].copy()
        self.assertEqual(bus.items['cup']['host'], 'person_arms')
        np.testing.assert_allclose(host_pose @ offset, object_pose, atol=1e-9)

        # The first update after handover must preserve the exact world pose.
        sent = []
        bus._send_pose = lambda op, model, pose: sent.append(
            (op, model, np.asarray(pose, float).copy()))
        bus.update()
        self.assertEqual(len(sent), 1)
        self.assertEqual(sent[0][0:2], ('set', 'cup'))
        np.testing.assert_allclose(sent[0][2], object_pose, atol=1e-9)

        # Subsequent host motion is transferred rigidly using the frozen offset.
        host_pose = K.trans((0.7, 0.05, 1.0)) @ K.frame(
            K.rot((0.0, 0.0, 1.0), 0.35))
        bus.update()
        np.testing.assert_allclose(
            sent[-1][2], host_pose @ offset, atol=1e-9)

        # A missing host observation must not overwrite the existing owner.
        previous = bus.items['cup']
        bus.host_world = lambda *_args: None
        self.assertFalse(bus.hand_to('cup', 'missing_host'))
        self.assertIs(bus.items['cup'], previous)


    def test_table_collision_boxes_match_work_area_sdf(self):
        world_path = os.path.join(
            ROOT, 'vla_ws', 'src', 'vla_follow', 'worlds', 'work_area.sdf')
        root = ET.parse(world_path).getroot()
        table = root.find(".//world/model[@name='table']")
        self.assertIsNotNone(table)
        model_pose = [float(x) for x in table.findtext('pose').split()]
        cx, cy, cz = model_pose[:3]
        self.assertAlmostEqual(cx, M.L.TABLE_XY[0], places=6)
        self.assertAlmostEqual(cy, M.L.TABLE_XY[1], places=6)
        self.assertAlmostEqual(model_pose[5], M.L.TABLE_YAW, places=6)

        link = table.find('link')
        top = link.find("./collision[@name='top']")
        top_pose = [float(x) for x in top.findtext('pose').split()]
        top_size = [float(x) for x in top.findtext('geometry/box/size').split()]
        self.assertAlmostEqual(
            cz + top_pose[2] + top_size[2] / 2.0,
            M.L.TABLE_TOP_Z, places=6)
        self.assertAlmostEqual(
            top_size[0], M.L.TABLE_W, places=6)
        self.assertAlmostEqual(
            top_size[1], M.L.TABLE_D, places=6)
        self.assertAlmostEqual(
            top_size[2], M.L.TABLE_PLATE_T, places=6)

        leg_collisions = [
            item for item in link.findall('collision')
            if item.attrib.get('name', '').startswith('leg')]
        self.assertEqual(len(leg_collisions), 4)
        expected_offsets = {
            (-0.410, -0.270), (-0.410, 0.270),
            (0.410, -0.270), (0.410, 0.270)}
        actual_offsets = set()
        for leg in leg_collisions:
            pose = [float(x) for x in leg.findtext('pose').split()]
            size = [float(x) for x in leg.findtext('geometry/box/size').split()]
            actual_offsets.add((round(pose[0], 3), round(pose[1], 3)))
            self.assertAlmostEqual(size[0], M.L.TABLE_LEG, places=6)
            self.assertAlmostEqual(size[1], M.L.TABLE_LEG, places=6)
            self.assertAlmostEqual(
                cz + pose[2] + size[2] / 2.0,
                M.L.TABLE_TOP_Z - M.L.TABLE_PLATE_T, places=6)
        self.assertEqual(actual_offsets, expected_offsets)

        boxes = {name: (np.asarray(lo), np.asarray(hi))
                 for name, lo, hi in M._table_collision_boxes()}
        expected_top_lo = np.array([
            cx - M.L.TABLE_W / 2.0,
            cy - M.L.TABLE_D / 2.0,
            M.L.TABLE_TOP_Z - M.L.TABLE_PLATE_T])
        expected_top_hi = np.array([
            cx + M.L.TABLE_W / 2.0,
            cy + M.L.TABLE_D / 2.0,
            M.L.TABLE_TOP_Z])
        np.testing.assert_allclose(boxes['table_top'][0], expected_top_lo)
        np.testing.assert_allclose(boxes['table_top'][1], expected_top_hi)

        # The conservative lip AABB must fully contain the visual lip in SDF.
        lip = link.find("./visual[@name='v_lip']")
        lip_pose = [float(x) for x in lip.findtext('pose').split()]
        lip_size = np.array([
            float(x) for x in lip.findtext('geometry/box/size').split()])
        lip_center = np.array([cx, cy, cz]) + np.asarray(lip_pose[:3])
        lip_lo, lip_hi = lip_center - lip_size / 2.0, lip_center + lip_size / 2.0
        box_lo, box_hi = boxes['table_lip']
        self.assertTrue(np.all(box_lo <= lip_lo + 1e-9))
        self.assertTrue(np.all(box_hi >= lip_hi - 1e-9))

        # Four collision boxes must match the four SDF leg centers and size.
        leg_boxes = [
            (np.asarray(lo, float), np.asarray(hi, float))
            for name, lo, hi in M._table_collision_boxes()
            if name == 'table_leg']
        self.assertEqual(len(leg_boxes), 4)
        for ox, oy in expected_offsets:
            center = np.array([cx + ox, cy + oy])
            matches = [
                (lo, hi) for lo, hi in leg_boxes
                if np.allclose((lo[:2] + hi[:2]) / 2.0, center)]
            self.assertEqual(len(matches), 1)
            lo, hi = matches[0]
            np.testing.assert_allclose(hi[:2] - lo[:2], M.L.TABLE_LEG)
            self.assertAlmostEqual(lo[2], 0.0, places=6)
            self.assertAlmostEqual(
                hi[2], M.L.TABLE_TOP_Z - M.L.TABLE_PLATE_T, places=6)

    def test_hang_final_settle_is_collision_gated_again(self):
        class Logger:
            def info(self, *_args):
                pass
            def warn(self, *_args):
                pass
            def error(self, *_args):
                pass

        class Node:
            def __init__(self):
                self.base = (0.0, 0.0, 0.0)
                self.js = {
                    'L_%s_joint' % joint[0]: float(value)
                    for joint, value in zip(K.JOINTS, (
                        0.0, 0.0, 0.0, K.HANG_FOLD, 0.0, 0.0, 0.0))}
                self.arm = {'L': Arm(self)}
                self.hand = {'L': Hand()}
            def get_logger(self):
                return Logger()

        class Arm:
            def __init__(self, node):
                self.node = node
            def move(self, q, **_kwargs):
                for joint, value in zip(K.JOINTS, q):
                    self.node.js['L_%s_joint' % joint[0]] = float(value)
                return True, np.zeros(K.NJ)

        class Hand:
            def move(self, _q, **_kwargs):
                return True, np.zeros(1)

        node = Node()
        gate_calls = []

        def allow_path(_node, side, path, **_kwargs):
            gate_calls.append((side, [np.asarray(q, float).copy() for q in path]))
            return True, None

        with patch.object(M, 'arm_path_table_clear', side_effect=allow_path):
            q_after, result = M.hang_natural(
                node, 'L', stages=(3,), check_table=True)

        self.assertTrue(result['ok'])
        self.assertEqual(result['stages_done'], [3])
        # One guard for the planned stage-3 path and another for the final
        # measured-state -> Q_HANG settling correction.
        self.assertEqual(len(gate_calls), 2)
        np.testing.assert_allclose(gate_calls[-1][1][-1], K.Q_HANG)
        np.testing.assert_allclose(q_after, K.Q_HANG)

    def test_hand_collision_proxy_dimensions_match_work_area_sdf(self):
        world_path = os.path.join(
            ROOT, 'vla_ws', 'src', 'vla_follow', 'worlds', 'work_area.sdf')
        root = ET.parse(world_path).getroot()
        robot = root.find(".//world/model[@name='robot']")
        self.assertIsNotNone(robot)

        finger_names = ('index', 'middle', 'ring', 'little')
        for side in ('R', 'L'):
            palm = robot.find("./link[@name='%s_palm']" % side)
            self.assertIsNotNone(palm)
            palm_pose = [float(x) for x in palm.findtext('pose').split()]
            self.assertAlmostEqual(palm_pose[2], -K.PALM_OFF, places=6)

            palm_visual = palm.find("./visual[@name='palm']")
            palm_visual_pose = [
                float(x) for x in palm_visual.findtext('pose').split()]
            palm_size = [
                float(x) for x in
                palm_visual.findtext('geometry/box/size').split()]
            np.testing.assert_allclose(
                palm_visual_pose[:3], K.PALM_CENTER, atol=1e-9)
            np.testing.assert_allclose(
                palm_size, K.PALM_SIZE, atol=1e-9)

            for index, finger in enumerate(finger_names):
                link = robot.find(
                    "./link[@name='%s_hand_%s_1']" % (side, finger))
                self.assertIsNotNone(link)
                pose = [float(x) for x in link.findtext('pose').split()]
                self.assertAlmostEqual(
                    pose[0], K.FINGER_X[index], places=6)
                self.assertAlmostEqual(
                    pose[1], 0.0, places=6)
                self.assertAlmostEqual(
                    pose[2], K.FINGER_BASE_Z, places=6)
                proximal = [
                    float(x) for x in
                    link.findtext("./visual[@name='v']/geometry/box/size")
                    .split()]
                self.assertAlmostEqual(
                    proximal[0], K.FINGER_W, places=6)
                self.assertAlmostEqual(
                    proximal[2], K.PROX_LEN, places=6)

                distal_link = robot.find(
                    "./link[@name='%s_hand_%s_2']" % (side, finger))
                self.assertIsNotNone(distal_link)
                distal_pose = [
                    float(x) for x in distal_link.findtext('pose').split()]
                self.assertAlmostEqual(
                    distal_pose[2], -K.PROX_LEN, places=6)
                distal = [
                    float(x) for x in
                    distal_link.findtext("./visual[@name='v']/geometry/box/size")
                    .split()]
                self.assertAlmostEqual(
                    distal[2], K.DIST_LEN, places=6)
                self.assertLessEqual(distal[0], K.FINGER_W)

            thumb = robot.find(
                "./link[@name='%s_hand_thumb_1']" % side)
            self.assertIsNotNone(thumb)
            thumb_pose = [
                float(x) for x in thumb.findtext('pose').split()]
            self.assertAlmostEqual(
                thumb_pose[0], K.THUMB_X[side], places=6)
            self.assertAlmostEqual(
                thumb_pose[1], K.THUMB_BASE[1], places=6)
            self.assertAlmostEqual(
                thumb_pose[2], K.THUMB_BASE[2], places=6)
            thumb_visual = [
                float(x) for x in
                thumb.findtext("./visual[@name='v']/geometry/box/size")
                .split()]
            thumb_2 = robot.find(
                "./link[@name='%s_hand_thumb_2']" % side)
            thumb_2_pose = [
                float(x) for x in thumb_2.findtext('pose').split()]
            thumb_2_visual = [
                float(x) for x in
                thumb_2.findtext("./visual[@name='v']/geometry/box/size")
                .split()]
            # The path guard models the thumb as one conservative 60mm capsule;
            # the SDF's two visual links total 54mm, so it must not be shorter.
            self.assertAlmostEqual(
                thumb_2_pose[2], -thumb_visual[2], places=6)
            self.assertGreaterEqual(
                K.PROX_LEN + K.DIST_LEN,
                thumb_visual[2] + thumb_2_visual[2])

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
