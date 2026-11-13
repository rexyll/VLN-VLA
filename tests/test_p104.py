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
import math
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
                            finite_numeric_value, summarize_batch_status,
                            load_json_strict, cleanup_observer_resources,
                            identify_unknown_rounds, invalid_round_candidates)

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
from vla_follow import scene_layout as L  # noqa: E402


class TestP104(unittest.TestCase):

    def test_present_approach_threshold_catches_observed_34mm_cup_offset(self):
        # E2E #48 had the hand target 33.6mm beyond the 0.70m arm reach.
        # A 35mm deadband incorrectly skipped base compensation, leaving one
        # palm 123.9mm from the cup axis (acceptance maximum is 117mm).
        self.assertAlmostEqual(L.PRESENT_APPROACH_CLEAR, 0.69)
        self.assertLessEqual(L.PRESENT_APPROACH_GAP, 0.010)
        observed_hand_distance = L.PRESENT_APPROACH_CLEAR + 0.0336
        self.assertGreater(
            observed_hand_distance - L.PRESENT_APPROACH_CLEAR,
            L.PRESENT_APPROACH_GAP)

    def test_present_approach_small_correction_accounts_for_drive_tolerance(self):
        # E2E #62: the measured hand distance was just over 0.700m, while
        # CLEAR=0.690m and the accepted gap is 10mm. A 10mm command is inside
        # drive_to's 12mm arrival tolerance and may be reported as no movement.
        required = 0.0101
        command = L.present_approach_command_step(required, 0.34)
        self.assertGreaterEqual(command, 0.02)
        self.assertGreaterEqual(command - required, 0.017 - 1e-9)
        # The command remains bounded by the table-clearance result.
        self.assertLessEqual(command, 0.34)
        # If only 19mm of collision-clear travel remains, do not issue a
        # sub-floor motion; preserve fail-closed behavior.
        self.assertEqual(L.present_approach_command_step(required, 0.019), 0.0)
        # Larger shortfalls may use a safe partial step and be re-measured.
        self.assertAlmostEqual(
            L.present_approach_command_step(0.10, 0.025), 0.025)

    def test_handover_geometry_gate_is_fail_closed_at_existing_92_plusminus_25mm(self):
        self.assertTrue(M.person_handover_geometry_ok((67.0, 117.0)))
        self.assertTrue(M.person_handover_geometry_ok((92.0, 92.0)))
        self.assertFalse(M.person_handover_geometry_ok((66.9, 92.0)))
        self.assertFalse(M.person_handover_geometry_ok((92.0, 117.1)))
        self.assertFalse(M.person_handover_geometry_ok((62.0, 127.3)))
        self.assertFalse(M.person_handover_geometry_ok(None))
        self.assertFalse(M.person_handover_geometry_ok((float('nan'), 92.0)))

    def test_present_target_predicts_closed_articulated_palm_midpoint(self):
        # Build current palm transforms from known shoulder frames at an open
        # joint angle, then verify the predicted closed midpoint matches FK.
        q_open = 0.42
        q_close = 0.035
        T_left_link = K.trans((1.0, 2.0, 1.0)) @ K.frame(
            K.rot((0.0, 0.0, 1.0), q_open))
        T_right_link = K.trans((1.0, 2.0, 1.0)) @ K.frame(
            K.rot((0.0, 0.0, 1.0), -q_open))
        T_left = T_left_link @ K.trans(L.PERSON_HAND_IN_ARML)
        T_right = T_right_link @ K.trans(L.PERSON_HAND_IN_ARMR)
        predicted = M.person_hand_midpoint_predict(
            T_left, T_right, q_open, -q_open, q_close)
        expected_left = (K.trans((1.0, 2.0, 1.0)) @ K.frame(
            K.rot((0.0, 0.0, 1.0), q_close))
            @ K.trans(L.PERSON_HAND_IN_ARML))
        expected_right = (K.trans((1.0, 2.0, 1.0)) @ K.frame(
            K.rot((0.0, 0.0, 1.0), -q_close))
            @ K.trans(L.PERSON_HAND_IN_ARMR))
        expected = 0.5 * (expected_left[:3, 3] + expected_right[:3, 3])
        self.assertIsNotNone(predicted)
        np.testing.assert_allclose(predicted[:3, 3], expected, atol=1e-9)
        # The midpoint differs from the fixed shoulder-root nominal point;
        # this is the offset that the PRESENT target must now account for.
        self.assertGreater(
            float(np.linalg.norm(expected - np.array((1.2, 2.0, 0.98)))),
            1e-3)

    def test_person_hand_midpoint_uses_measured_truth_when_already_at_target(self):
        # Gazebo reports articulated palm links even when JointState omits
        # person_arm_l_joint/person_arm_r_joint. An already-reached target uses
        # the measured midpoint without requiring angle reconstruction.
        left = K.trans((1.0, 2.0 - 0.092, 1.02))
        right = K.trans((1.0, 2.0 + 0.092, 1.02))
        midpoint = M.person_hand_midpoint_if_at_target(
            left, right, L.PERSON_HAND_CLOSE)
        self.assertIsNotNone(midpoint)
        np.testing.assert_allclose(
            midpoint[:3, 3], np.array((1.0, 2.0, 1.02)), atol=1e-9)
        open_left = K.trans((1.0, 2.0 - 0.1725, 1.02))
        open_right = K.trans((1.0, 2.0 + 0.1725, 1.02))
        self.assertIsNone(M.person_hand_midpoint_if_at_target(
            open_left, open_right, L.PERSON_HAND_CLOSE))
        self.assertIsNotNone(M.person_hand_midpoint_if_at_target(
            open_left, open_right, L.PERSON_HAND_OPEN))

    def test_person_hand_midpoint_predicts_from_gap_confirmed_mirrored_command(self):
        q_open, q_close = 0.42, 0.035
        left_root = K.trans((1.0, 2.0 + 0.155, 1.0))
        right_root = K.trans((1.0, 2.0 - 0.155, 1.0))
        left_link = left_root @ K.frame(K.rot((0.0, 0.0, 1.0), q_open))
        right_link = right_root @ K.frame(K.rot((0.0, 0.0, 1.0), -q_open))
        left = left_link @ K.trans(L.PERSON_HAND_IN_ARML)
        right = right_link @ K.trans(L.PERSON_HAND_IN_ARMR)
        predicted = M.person_hand_midpoint_from_confirmed_command(
            left, right, q_open, -q_open, q_close)
        expected_l = left_root @ K.frame(K.rot((0.0, 0.0, 1.0), q_close)) @ K.trans(
            L.PERSON_HAND_IN_ARML)
        expected_r = right_root @ K.frame(K.rot((0.0, 0.0, 1.0), -q_close)) @ K.trans(
            L.PERSON_HAND_IN_ARMR)
        expected = 0.5 * (expected_l[:3, 3] + expected_r[:3, 3])
        self.assertIsNotNone(predicted)
        np.testing.assert_allclose(predicted[:3, 3], expected, atol=1e-9)
        self.assertIsNone(M.person_hand_midpoint_from_confirmed_command(
            left, right, q_open, q_open, q_close))
        wrong_gap = right.copy()
        wrong_gap[1, 3] += 0.05
        self.assertIsNone(M.person_hand_midpoint_from_confirmed_command(
            left, wrong_gap, q_open, -q_open, q_close))

    def test_reach_servo_failure_diagnostic_captures_both_joint_caches(self):
        source = open(os.path.join(ROOT, 'vla_ws', 'src', 'vla_follow', 'vla_follow', 'drink_motion.py'), encoding='utf-8').read()
        self.assertIn('q_servo_final = np.asarray(node.arm[side]._read()', source)
        self.assertIn("'servo_final_rad'", source)
        self.assertIn("'node_final_rad'", source)
        self.assertIn("'reported_error_rad'", source)
        self.assertIn('servo_measured=%s servo_error_deg=%s node_error_deg=%s', source)
        # Command arrays are copied before the servo call so downstream
        # mutation cannot rewrite the reference target in the failure record.
        self.assertIn('q_commanded = np.asarray(q, dtype=float).copy()', source)

    def test_present_target_emits_target_source_and_execution_residual_diagnostics(self):
        motion = open(os.path.join(ROOT, 'vla_ws', 'src', 'vla_follow',
                                  'vla_follow', 'drink_motion.py'),
                      encoding='utf-8').read()
        task = open(os.path.join(ROOT, 'vla_ws', 'src', 'vla_follow',
                                'vla_follow', 'drink_task.py'),
                    encoding='utf-8').read()
        self.assertIn('PRESENT_TARGET source=nominal_fallback', motion)
        self.assertIn('PRESENT_TARGET source=predicted_closed_midpoint', motion)
        self.assertIn('target_minus_hand_mm=%s', motion)
        self.assertIn('PRESENT_EXEC_RESULT ok=%s reported_mm=%.3f', task)
        self.assertIn('actual_position_mm=%s actual_axis_deg=%s', task)

    def test_present_target_prediction_rejects_missing_or_nonfinite_joint_state(self):
        valid = K.trans((0.0, 0.0, 0.0))
        self.assertIsNone(M.person_hand_midpoint_predict(
            valid, valid, None, 0.0, L.PERSON_HAND_CLOSE))
        self.assertIsNone(M.person_hand_midpoint_predict(
            valid, valid, 0.0, float('nan'), L.PERSON_HAND_CLOSE))


    def test_person_hand_midpoint_offset_tracks_measured_palms_in_rotated_arm_frame(self):
        T_arms = K.trans((0.4, -0.2, 0.8)) @ K.frame(
            K.rot((0.0, 0.0, 1.0), math.radians(37.0)))
        T_left = K.trans((0.9, 0.1, 1.0))
        T_right = K.trans((0.7, 0.3, 1.0))
        offset = M.person_hand_midpoint_offset(T_arms, T_left, T_right)
        self.assertIsNotNone(offset)
        reconstructed = T_arms @ K.trans(offset)
        expected = 0.5 * (T_left[:3, 3] + T_right[:3, 3])
        np.testing.assert_allclose(reconstructed[:3, 3], expected, atol=1e-9)

    def test_person_hand_midpoint_offset_rejects_missing_or_invalid_truth(self):
        valid = np.eye(4)
        self.assertIsNone(M.person_hand_midpoint_offset(None, valid, valid))
        invalid = valid.copy()
        invalid[0, 0] = np.nan
        self.assertIsNone(M.person_hand_midpoint_offset(valid, invalid, valid))
        singular = valid.copy()
        singular[:3, :3] = 0.0
        self.assertIsNone(M.person_hand_midpoint_offset(singular, valid, valid))

    def test_handover_geometry_gate_runs_before_host_ownership_transfer(self):
        task_path = os.path.join(
            ROOT, 'vla_ws', 'src', 'vla_follow', 'vla_follow',
            'drink_task.py')
        with open(task_path, encoding='utf-8') as stream:
            source = stream.read()
        start = source.index('_pre_hc = M.hands_to_cup_mm(self)')
        gate = source.index("self._set_state('FAIL_HANDOVER_GEOMETRY')", start)
        transfer = source.index("self.carry.hand_to('cup', 'person', 'person_arms'", start)
        diagnostic = source.index('HANDOVER_GEOM world_m')
        self.assertLess(start, gate)
        self.assertLess(diagnostic, gate)
        self.assertLess(gate, transfer)
        self.assertIn('M.person_hand_midpoint_offset(_tp @ _ta, _hl, _hr)', source)
        self.assertIn('_handover_off)', source[transfer:transfer + 180])
        self.assertIn('FAIL_HANDOVER_ANCHOR', source)
        self.assertIn('保持机器人持杯，不交权', source[start:transfer])

    def test_pose_history_interpolates_at_rgbd_capture_time_without_extrapolation(self):
        # Latest pose is intentionally newer than the image by 300ms. The
        # historical pose at image time must be used, not rejected as skew.
        history = [
            (10.00, 1.0, 2.0, math.radians(179.0)),
            (10.10, 2.0, 4.0, math.radians(-179.0)),
            (10.20, 3.0, 6.0, math.radians(-170.0)),
        ]
        pose = V.pose_at_timestamp(history, 10.05)
        self.assertIsNotNone(pose)
        self.assertAlmostEqual(pose[0], 10.05)
        self.assertAlmostEqual(pose[1], 1.5)
        self.assertAlmostEqual(pose[2], 3.0)
        self.assertAlmostEqual(abs(pose[3]), math.pi, places=6)
        self.assertIsNone(V.pose_at_timestamp(history, 9.80))
        self.assertIsNone(V.pose_at_timestamp(history, 10.50))

    def test_pose_history_rejects_gaps_beyond_existing_skew_gate(self):
        history = [(1.0, 0.0, 0.0, 0.0), (1.5, 1.0, 0.0, 0.0)]
        self.assertIsNone(V.pose_at_timestamp(history, 1.25))

    def test_person_vision_rejection_diagnostics_capture_candidate_geometry(self):
        vision_path = os.path.join(
            ROOT, 'vla_ws', 'src', 'vla_follow', 'vla_follow',
            'person_vision.py')
        with open(vision_path, encoding='utf-8') as stream:
            source = stream.read()
        self.assertIn("'reject_samples': []", source)
        self.assertIn("note_reject('shape_y'", source)
        self.assertIn("valid_depth=valid.size, depth_q30=dist", source)
        self.assertIn("'reject_samples={}'", source)
        self.assertIn("results[name]['diag']['reject_samples']", source)
        self.assertIn("len(diag['reject_samples']) < 4", source)
        self.assertIn("if not (0.22 <= asp <= 1.25)", source)
        self.assertIn("elif area < 250 or area > 18000", source)

    def test_person_color_masks_detect_dark_skirt_and_blue_torso_separately(self):
        hsv = np.array([[
            [0, 0, 40],       # dark skirt
            [110, 200, 180],  # blue torso
            [20, 200, 180],   # unrelated saturated object
            [110, 30, 180],   # low-saturation blue-like object
        ]], dtype=np.uint8)
        dark, blue = V.person_color_masks(hsv)
        self.assertEqual(dark[0].tolist(), [255, 0, 0, 0])
        self.assertEqual(blue[0].tolist(), [0, 255, 0, 0])

    def test_blue_torso_candidate_has_stricter_geometry_than_dark_skirt(self):
        vision_path = os.path.join(
            ROOT, 'vla_ws', 'src', 'vla_follow', 'vla_follow',
            'person_vision.py')
        with open(vision_path, encoding='utf-8') as stream:
            vision = stream.read()
        self.assertIn("elif area < 250 or area > 18000:", vision)
        self.assertIn("if not (0.22 <= asp <= 1.25):", vision)
        self.assertIn("if y + 0.5 * h > img_h * 0.78:", vision)
        self.assertIn("elif not (0.12 <= w_vis <= 0.48):", vision)

    def test_missing_round_with_command_marker_is_classified_unknown(self):
        with tempfile.TemporaryDirectory() as outdir:
            missing = [
                {'round': 1, 'file': 'drink_case_1.json'},
                {'round': 2, 'file': 'drink_case_2.json'},
                {'round': 3, 'file': 'drink_case_3.json'},
            ]
            with open(os.path.join(outdir, 'drink_case_1.started'), 'w') as stream:
                stream.write('attempted')
            # A dangling marker path is still conservative evidence that the
            # round cannot be treated as definitely not attempted.
            os.symlink(os.path.join(outdir, 'not-created'),
                       os.path.join(outdir, 'drink_case_2.started'))
            # A malformed result file is not "missing", but its command
            # outcome is still unknown when the durable attempt marker exists.
            with open(os.path.join(outdir, 'drink_case_3.started'), 'w') as stream:
                stream.write('attempted')

            invalid_result_candidate = [
                {'round': 3, 'file': 'drink_case_3.json', 'error': 'invalid JSON'}
            ]
            unknown = identify_unknown_rounds(
                outdir, 'case', missing[:2] + invalid_result_candidate)
            self.assertEqual(unknown, missing[:2] + invalid_result_candidate)
            self.assertEqual(
                identify_unknown_rounds(outdir, 'case', [
                    {'round': True, 'file': 'invalid'},
                    {'round': 0, 'file': 'invalid'},
                    None,
                ]),
                [])



    def test_invalid_result_filenames_map_to_unknown_round_candidates(self):
        invalid = [
            {'file': 'drink_case_1.json', 'error': 'invalid JSON'},
            {'file': 'drink_case_3.json', 'error': 'non-object JSON'},
            {'file': '../drink_case_2.json', 'error': 'untrusted path'},
            {'file': 'unrelated.json', 'error': 'unrelated'},
            None,
        ]
        candidates = invalid_round_candidates(3, 'case', invalid)
        self.assertEqual(candidates, [
            {'round': 1, 'file': 'drink_case_1.json'},
            {'round': 3, 'file': 'drink_case_3.json'},
        ])

        with tempfile.TemporaryDirectory() as outdir:
            with open(os.path.join(outdir, 'drink_case_1.started'), 'w') as stream:
                stream.write('attempted')
            with open(os.path.join(outdir, 'drink_case_2.started'), 'w') as stream:
                stream.write('attempted')
            unknown = identify_unknown_rounds(outdir, 'case', candidates)
            self.assertEqual(unknown, [candidates[0]])

    def test_batch_status_counts_invalid_and_missing_rounds_as_failures(self):
        passed = {'ok': True}
        failed = {'ok': False}
        status = summarize_batch_status(
            3, [passed, failed], [{'file': 'round_3.json', 'error': 'invalid JSON'}])
        self.assertEqual(status['requested_rounds'], 3)
        self.assertEqual(status['completed_rounds'], 2)
        self.assertEqual(status['passed_rounds'], 1)
        self.assertEqual(status['failed_rounds'], 2)
        self.assertEqual(status['invalid_rounds'], [
            {'file': 'round_3.json', 'error': 'invalid JSON'}])
        self.assertFalse(status['all_passed'])

        complete = summarize_batch_status(2, [passed, passed], [])
        self.assertEqual(complete['failed_rounds'], 0)
        self.assertTrue(complete['all_passed'])

        missing = summarize_batch_status(
            3, [passed], [], [
                {'round': 2, 'file': 'drink_case_2.json'},
                {'round': 3, 'file': 'drink_case_3.json'},
            ])
        self.assertEqual(missing['failed_rounds'], 2)
        self.assertEqual(missing['missing_rounds'], [
            {'round': 2, 'file': 'drink_case_2.json'},
            {'round': 3, 'file': 'drink_case_3.json'},
        ])
        self.assertFalse(missing['all_passed'])


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

    def test_json_loading_rejects_nonstandard_numeric_constants(self):
        with tempfile.TemporaryDirectory() as root:
            target = os.path.join(root, 'round.json')
            for payload in ('{"metric": NaN}', '{"metric": Infinity}',
                            '{"nested": [-Infinity]}'):
                with open(target, 'w', encoding='utf-8') as stream:
                    stream.write(payload)
                with self.assertRaisesRegex(ValueError, 'non-standard JSON'):
                    load_json_strict(target)

            with open(target, 'w', encoding='utf-8') as stream:
                stream.write('{"metric": 1.25}')
            self.assertEqual(load_json_strict(target), {'metric': 1.25})

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

    def test_repeat_runner_sources_ros_with_nounset_temporarily_disabled(self):
        runner_path = os.path.join(ROOT, 'tests', 'drink_repeat.sh')
        with open(runner_path, encoding='utf-8') as stream:
            runner = stream.read()
        disable = runner.index('set +u')
        ros_setup = runner.index('source /opt/ros/jazzy/setup.bash', disable)
        overlay_setup = runner.index('source "$ROOT/vla_ws/install/setup.bash"', ros_setup)
        restore = runner.index('set -u', overlay_setup)
        self.assertLess(disable, ros_setup)
        self.assertLess(ros_setup, overlay_setup)
        self.assertLess(overlay_setup, restore)
        self.assertIn('AMENT_TRACE_SETUP_FILES', runner)

    def test_repeat_output_directory_rejects_symlink_escape(self):
        runner_path = os.path.join(ROOT, 'tests', 'drink_repeat.sh')
        with open(runner_path, encoding='utf-8') as stream:
            runner = stream.read()
        guard = runner.index('if [[ -L "$OUTDIR" ]]')
        create = runner.index('mkdir -p "$OUTDIR"', guard)
        resolve = runner.index('OUTDIR_REAL="$(realpath -e "$OUTDIR")"', create)
        containment = runner.index('"$TESTS_REAL"/*)', resolve)
        lock = runner.index('LOCKFILE="$OUTDIR/.drink_repeat.lock"')
        self.assertLess(guard, create)
        self.assertLess(create, resolve)
        self.assertLess(resolve, containment)
        self.assertLess(containment, lock)
        self.assertIn('DrinkTask output directory must not be a symlink', runner)
        self.assertIn('DrinkTask output directory escapes tests/', runner)

    def test_repeat_lock_file_rejects_symlink_and_avoids_truncation(self):
        runner_path = os.path.join(ROOT, 'tests', 'drink_repeat.sh')
        with open(runner_path, encoding='utf-8') as stream:
            runner = stream.read()
        guard = runner.index('if [[ -L "$LOCKFILE" ]]')
        open_lock = runner.index('exec 9>>"$LOCKFILE"', guard)
        acquire = runner.index('flock -n 9', open_lock)
        self.assertLess(guard, open_lock)
        self.assertLess(open_lock, acquire)
        self.assertIn('DrinkTask lock file must not be a symlink', runner)
        self.assertNotIn('exec 9>"$OUTDIR/.drink_repeat.lock"', runner)

    def test_repeat_summary_requires_runner_exit_success(self):
        runner_path = os.path.join(ROOT, 'tests', 'drink_repeat.sh')
        with open(runner_path, encoding='utf-8') as stream:
            runner = stream.read()
        self.assertIn('summary_path, requested, root, failed, tag, outdir, *paths = sys.argv[1:]', runner)
        self.assertIn('shot_dir, observer_exit_code = sys.argv[1:]', runner)
        self.assertIn('result["observer_exit_code"] = int(observer_exit_code)', runner)
        self.assertIn('validate_observer_result(result)', runner)
        self.assertIn('from drink_evidence import load_json_strict, summarize_round_evidence', runner)
        gate_start = runner.index('if ! python3 - "$SNAPSHOT" "$ROOT"')
        gate_end = runner.index('\nPY\n        then', gate_start)
        acceptance_gate = runner[gate_start:gate_end]
        self.assertIn('result = load_json_strict(sys.argv[1])', acceptance_gate)
        self.assertNotIn('json.load(', acceptance_gate)
        self.assertLess(
            runner.index('validate_observer_result(result)'),
            runner.index('result["observer_exit_code"] = int(observer_exit_code)'))
        self.assertIn('"$RC" <<\'PY\'', runner)
        self.assertIn('"process_ok": evidence["process_ok"]', runner)
        self.assertIn('missing_rounds = [', runner)
        self.assertIn('summarize_batch_status(', runner)
        self.assertIn('int(requested), rows, invalid_rounds, missing_rounds', runner)
        self.assertIn('**batch_status', runner)
        self.assertIn('invalid_rounds = []', runner)
        self.assertIn('invalid_rounds.append(', runner)
        self.assertIn('"runner_failed": int(failed) != 0', runner)
        self.assertIn('"all_passed": batch_status["all_passed"] and int(failed) == 0', runner)

        final_check_start = runner.rindex('python3 - "$SUMMARY" "$ROOT"')
        final_check_end = runner.index('\nPY', final_check_start)
        final_check = runner[final_check_start:final_check_end]
        self.assertIn('from drink_evidence import load_json_strict', final_check)
        self.assertIn('result = load_json_strict(summary_path)', final_check)
        self.assertNotIn('json.load(', final_check)

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

            # Distinct archive filenames must not disguise repeated source
            # evidence. Realpath also catches symlink aliases to one image.
            duplicate_target = os.path.join(root, 'archive', 'duplicate_shots')
            with self.assertRaisesRegex(ValueError, 'duplicate screenshot source'):
                archive_observer_screenshots(
                    {'shots': [sources[0], sources[0]]}, root, duplicate_target)
            self.assertFalse(os.path.lexists(duplicate_target))

            alias = os.path.join(source_dir, 'alias.png')
            os.symlink(sources[0], alias)
            alias_target = os.path.join(root, 'archive', 'alias_shots')
            with self.assertRaisesRegex(ValueError, 'duplicate screenshot source'):
                archive_observer_screenshots(
                    {'shots': [sources[0], alias]}, root, alias_target)
            self.assertFalse(os.path.lexists(alias_target))
            leftovers = [name for name in os.listdir(os.path.dirname(alias_target))
                         if name.startswith('.drink-shots-')]
            self.assertEqual(leftovers, [])

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
                'observer_exit_code': 0,
            }
            summary = summarize_round_evidence(result, root)
            self.assertTrue(summary['ok'])
            self.assertTrue(summary['archive_ok'])
            self.assertTrue(summary['process_ok'])
            self.assertEqual(summary['screenshots_archived'], 7)

            # Optional event-linked frames (such as the post-handover
            # following image) must not invalidate the seven required frames.
            optional_path = os.path.join(shot_dir, '08_following.png')
            with open(optional_path, 'wb') as stream:
                stream.write(b'optional-png-placeholder')
            optional_result = dict(
                result,
                shots=shots + [os.path.relpath(optional_path, root)],
                shot_labels=labels + ['following'],
                evidence={'screenshots_archived': 8})
            optional_summary = summarize_round_evidence(optional_result, root)
            self.assertTrue(optional_summary['ok'])
            self.assertTrue(optional_summary['archive_ok'])
            self.assertEqual(optional_summary['screenshots_archived'], 8)

            # A complete task JSON and screenshot archive must still fail
            # acceptance when the observer process exited non-zero.
            crashed_observer = dict(result, observer_exit_code=7)
            crashed_summary = summarize_round_evidence(crashed_observer, root)
            self.assertFalse(crashed_summary['process_ok'])
            self.assertFalse(crashed_summary['ok'])
            self.assertEqual(crashed_summary['observer_exit_code'], 7)

            # Missing, boolean, and string process statuses are not proof of
            # a clean observer exit, even when task JSON and screenshots pass.
            for invalid_exit_code in (None, True, '0'):
                invalid_process = dict(result, observer_exit_code=invalid_exit_code)
                invalid_summary = summarize_round_evidence(invalid_process, root)
                self.assertFalse(invalid_summary['process_ok'])
                self.assertFalse(invalid_summary['ok'])

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


    def test_command_marker_fdopen_failure_closes_descriptor_and_preserves_error(self):
        observer_path = os.path.join(ROOT, 'tests', 'test_drink.py')
        with open(observer_path, encoding='utf-8') as stream:
            source = stream.read()
        import ast
        tree = ast.parse(source)
        helper = next(node for node in tree.body
                      if isinstance(node, ast.FunctionDef)
                      and node.name == '_create_command_attempt_marker')
        isolated = ast.Module(body=[helper], type_ignores=[])
        namespace = {'os': os}
        exec(compile(isolated, observer_path, 'exec'), namespace)

        original_error = OSError('fdopen injected failure')
        with patch('os.open', return_value=731) as open_file:
            with patch('os.fdopen', side_effect=original_error):
                with patch('os.close') as close_file:
                    with self.assertRaises(OSError) as caught:
                        namespace['_create_command_attempt_marker'](
                            '/tmp/unused-marker')
        self.assertIs(caught.exception, original_error)
        open_file.assert_called_once()
        close_file.assert_called_once_with(731)

        # The command wrapper must not publish when marker creation fails.
        publish = next(node for node in tree.body
                       if isinstance(node, ast.FunctionDef)
                       and node.name == '_publish_drink_command')
        isolated_publish = ast.Module(body=[helper, publish], type_ignores=[])
        namespace = {'os': os}
        exec(compile(isolated_publish, observer_path, 'exec'), namespace)

        class Watch:
            def __init__(self):
                self.sent = []
            def send(self, action):
                self.sent.append(action)

        watch = Watch()
        with patch('os.open', return_value=732):
            with patch('os.fdopen', side_effect=original_error):
                with patch('os.close') as close_file:
                    with self.assertRaises(OSError) as publish_error:
                        namespace['_publish_drink_command'](
                            watch, '/tmp/unused-marker')
        self.assertIs(publish_error.exception, original_error)
        close_file.assert_called_once_with(732)
        self.assertEqual(watch.sent, [])


    def test_existing_command_marker_blocks_go_drink_publication(self):
        observer_path = os.path.join(ROOT, 'tests', 'test_drink.py')
        with open(observer_path, encoding='utf-8') as stream:
            source = stream.read()
        import ast
        tree = ast.parse(source)
        helper = next(node for node in tree.body
                      if isinstance(node, ast.FunctionDef)
                      and node.name == '_create_command_attempt_marker')
        publish = next(node for node in tree.body
                       if isinstance(node, ast.FunctionDef)
                       and node.name == '_publish_drink_command')
        isolated = ast.Module(body=[helper, publish], type_ignores=[])
        namespace = {'os': os}
        exec(compile(isolated, observer_path, 'exec'), namespace)

        class Watch:
            def __init__(self):
                self.sent = []
            def send(self, action):
                self.sent.append(action)

        watch = Watch()
        collision = FileExistsError('marker already exists')
        with patch('os.open', side_effect=collision) as open_file:
            with patch('os.fdopen') as fdopen_file:
                with self.assertRaises(FileExistsError) as caught:
                    namespace['_publish_drink_command'](
                        watch, '/tmp/existing-marker')
        self.assertIs(caught.exception, collision)
        open_file.assert_called_once()
        self.assertEqual(open_file.call_args.args[1],
                         os.O_WRONLY | os.O_CREAT | os.O_EXCL)
        self.assertEqual(open_file.call_args.args[2], 0o600)
        fdopen_file.assert_not_called()
        self.assertEqual(watch.sent, [])

    def test_command_marker_success_flushes_and_fsyncs(self):
        observer_path = os.path.join(ROOT, 'tests', 'test_drink.py')
        with open(observer_path, encoding='utf-8') as stream:
            source = stream.read()
        import ast
        tree = ast.parse(source)
        helper = next(node for node in tree.body
                      if isinstance(node, ast.FunctionDef)
                      and node.name == '_create_command_attempt_marker')
        isolated = ast.Module(body=[helper], type_ignores=[])
        namespace = {'os': os}
        exec(compile(isolated, observer_path, 'exec'), namespace)

        class Stream:
            def __init__(self):
                self.writes = []
                self.flushed = False
            def __enter__(self):
                return self
            def __exit__(self, *_args):
                return False
            def write(self, value):
                self.writes.append(value)
            def flush(self):
                self.flushed = True
            def fileno(self):
                return 731

        stream = Stream()
        with patch('os.open', side_effect=[731, 732]) as open_file:
            with patch('os.fdopen', return_value=stream):
                with patch('os.fsync') as sync_file:
                    with patch('os.close') as close_file:
                        namespace['_create_command_attempt_marker'](
                            '/tmp/unused-marker')
        self.assertEqual(stream.writes, ['GO_DRINK command attempt started\\n'])
        self.assertTrue(stream.flushed)
        self.assertEqual([call.args for call in open_file.call_args_list], [
            ('/tmp/unused-marker', os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600),
            ('/tmp', os.O_RDONLY | getattr(os, 'O_DIRECTORY', 0)),
        ])
        self.assertEqual([call.args for call in sync_file.call_args_list],
                         [(731,), (732,)])
        close_file.assert_called_once_with(732)

    def test_publish_drink_command_success_persists_marker_before_send(self):
        observer_path = os.path.join(ROOT, 'tests', 'test_drink.py')
        with open(observer_path, encoding='utf-8') as stream:
            source = stream.read()
        import ast
        tree = ast.parse(source)
        helper = next(node for node in tree.body
                      if isinstance(node, ast.FunctionDef)
                      and node.name == '_create_command_attempt_marker')
        publish = next(node for node in tree.body
                       if isinstance(node, ast.FunctionDef)
                       and node.name == '_publish_drink_command')
        isolated = ast.Module(body=[helper, publish], type_ignores=[])
        events = []

        class MarkerStream:
            def __enter__(self):
                return self
            def __exit__(self, *_args):
                events.append('close')
                return False
            def write(self, value):
                events.append(('write', value))
            def flush(self):
                events.append('flush')
            def fileno(self):
                return 741

        class Watch:
            def send(self, action):
                events.append(('send', action))

        namespace = {'os': os}
        exec(compile(isolated, observer_path, 'exec'), namespace)
        marker_stream = MarkerStream()
        def open_marker_or_directory(path, *_args):
            events.append(('open', path))
            return 741 if path == '/tmp/unused-marker' else 742

        with patch('os.open', side_effect=open_marker_or_directory):
            with patch('os.fdopen',
                       side_effect=lambda *_args, **_kwargs:
                       events.append('fdopen') or marker_stream):
                with patch('os.fsync',
                           side_effect=lambda fd: events.append(('fsync', fd))):
                    with patch('os.close',
                               side_effect=lambda fd: events.append(('close-dir', fd))):
                        namespace['_publish_drink_command'](
                            Watch(), '/tmp/unused-marker')
        self.assertEqual(events, [
            ('open', '/tmp/unused-marker'), 'fdopen',
            ('write', 'GO_DRINK command attempt started\\n'),
            'flush', ('fsync', 741), 'close',
            ('open', '/tmp'), ('fsync', 742), ('close-dir', 742),
            ('send', 'GO_DRINK')])

        # An absent marker path is an intentional opt-out, not a send failure.
        events.clear()
        namespace['_publish_drink_command'](Watch(), None)
        self.assertEqual(events, [('send', 'GO_DRINK')])

    def test_publish_failure_after_durable_marker_leaves_attempt_evidence(self):
        observer_path = os.path.join(ROOT, 'tests', 'test_drink.py')
        with open(observer_path, encoding='utf-8') as stream:
            source = stream.read()
        import ast
        tree = ast.parse(source)
        helper = next(node for node in tree.body
                      if isinstance(node, ast.FunctionDef)
                      and node.name == '_create_command_attempt_marker')
        publish = next(node for node in tree.body
                       if isinstance(node, ast.FunctionDef)
                       and node.name == '_publish_drink_command')
        isolated = ast.Module(body=[helper, publish], type_ignores=[])
        events = []
        send_error = RuntimeError('injected command transport failure')

        class MarkerStream:
            def __enter__(self):
                return self
            def __exit__(self, *_args):
                events.append('close')
                return False
            def write(self, value):
                events.append(('write', value))
            def flush(self):
                events.append('flush')
            def fileno(self):
                return 751

        class Watch:
            def send(self, action):
                events.append(('send', action))
                raise send_error

        namespace = {'os': os}
        exec(compile(isolated, observer_path, 'exec'), namespace)
        marker_stream = MarkerStream()
        def open_marker_or_directory(path, *_args):
            events.append(('open', path))
            return 751 if path == '/tmp/unused-marker' else 752

        with patch('os.open', side_effect=open_marker_or_directory):
            with patch('os.fdopen',
                       side_effect=lambda *_args, **_kwargs:
                       events.append('fdopen') or marker_stream):
                with patch('os.fsync',
                           side_effect=lambda fd: events.append(('fsync', fd))):
                    with patch('os.close',
                               side_effect=lambda fd: events.append(('close-dir', fd))):
                        with self.assertRaises(RuntimeError) as caught:
                            namespace['_publish_drink_command'](
                                Watch(), '/tmp/unused-marker')

        self.assertIs(caught.exception, send_error)
        self.assertEqual(events, [
            ('open', '/tmp/unused-marker'), 'fdopen',
            ('write', 'GO_DRINK command attempt started\\n'),
            'flush', ('fsync', 751), 'close',
            ('open', '/tmp'), ('fsync', 752), ('close-dir', 752),
            ('send', 'GO_DRINK')])

    def test_drink_observer_owns_ros_cleanup_in_main_finally(self):
        observer_path = os.path.join(ROOT, 'tests', 'test_drink.py')
        with open(observer_path, encoding='utf-8') as stream:
            source = stream.read()

        main_start = source.index('def main():')
        main_end = source.index("\n\nif __name__ == '__main__':", main_start)
        main_body = source[main_start:main_end]
        self.assertIn('try:', main_body)
        self.assertIn('finally:', main_body)
        self.assertIn('node = DrinkTask(observer_only=True)', main_body)
        self.assertIn('result = _run_observer(node)', main_body)
        self.assertIn('cleanup_observer_resources(', main_body)
        self.assertIn('if cleanup_failed and result == 0:', main_body)
        self.assertIn('return 3', main_body)
        self.assertNotIn('node.destroy_node()', main_body)
        self.assertNotIn('rclpy.shutdown()', main_body)

        report_start = source.index('def report(node, res):')
        report_end = source.index('\n\ndef main():', report_start)
        report_body = source[report_start:report_end]
        self.assertNotIn('destroy_node()', report_body)
        self.assertNotIn('rclpy.shutdown()', report_body)

        runner_path = os.path.join(ROOT, 'tests', 'drink_repeat.sh')
        with open(runner_path, encoding='utf-8') as stream:
            runner = stream.read()
        self.assertIn('UNKNOWN/INTERRUPTED', runner)
        self.assertIn('FAILED=1', runner)
        self.assertIn('P91_COMMAND_MARKER', runner)
        self.assertIn('UNKNOWN/INTERRUPTED', runner)
        self.assertIn('"unknown_rounds": unknown_rounds', runner)
        self.assertIn('P91_COMMAND_MARKER', source)
        self.assertIn('os.O_EXCL', source)
        self.assertIn('_publish_drink_command(w, marker_path)', source)
        publish_start = source.index('def _publish_drink_command(')
        publish_end = source.index('\n\ndef _run_observer(', publish_start)
        publish_body = source[publish_start:publish_end]
        self.assertLess(
            publish_body.index('_create_command_attempt_marker(marker_path)'),
            publish_body.index("watch.send('GO_DRINK')"))
        marker_start = source.index('def _create_command_attempt_marker(')
        marker_end = source.index('\n\ndef _run_observer(', marker_start)
        marker_body = source[marker_start:marker_end]
        self.assertIn('except BaseException:', marker_body)
        self.assertIn('os.close(fd)', marker_body)
        self.assertIn('os.fsync(stream.fileno())', marker_body)
        self.assertIn('os.open(directory, directory_flags)', marker_body)
        self.assertIn('os.fsync(directory_fd)', marker_body)
        self.assertIn('os.close(directory_fd)', marker_body)


    def test_marker_directory_fsync_failure_blocks_go_drink(self):
        observer_path = os.path.join(ROOT, 'tests', 'test_drink.py')
        with open(observer_path, encoding='utf-8') as stream:
            source = stream.read()
        import ast
        tree = ast.parse(source)
        helper = next(node for node in tree.body
                      if isinstance(node, ast.FunctionDef)
                      and node.name == '_create_command_attempt_marker')
        publish = next(node for node in tree.body
                       if isinstance(node, ast.FunctionDef)
                       and node.name == '_publish_drink_command')
        isolated = ast.Module(body=[helper, publish], type_ignores=[])
        namespace = {'os': os}
        exec(compile(isolated, observer_path, 'exec'), namespace)
        error = OSError('directory fsync injected failure')
        calls = []

        class Stream:
            def __enter__(self):
                return self
            def __exit__(self, *_args):
                calls.append('close-file')
                return False
            def write(self, _value):
                calls.append('write')
            def flush(self):
                calls.append('flush')
            def fileno(self):
                return 761

        class Watch:
            def send(self, action):
                calls.append(('send', action))

        with patch('os.open', side_effect=[761, 762]):
            with patch('os.fdopen', return_value=Stream()):
                with patch('os.fsync', side_effect=[None, error]):
                    with patch('os.close',
                               side_effect=lambda fd: calls.append(('close-dir', fd))):
                        with self.assertRaises(OSError) as caught:
                            namespace['_publish_drink_command'](
                                Watch(), '/tmp/unused-marker')
        self.assertIs(caught.exception, error)
        self.assertEqual(calls, [
            'write', 'flush', 'close-file', ('close-dir', 762)])
        calls = []

        class Node:
            def destroy_node(self):
                calls.append('destroy')

        failed = cleanup_observer_resources(
            Node(), True,
            lambda: calls.append('ok') or True,
            lambda: calls.append('shutdown'),
            lambda label, exc: self.fail('unexpected cleanup error: %s' % label))
        self.assertFalse(failed)
        self.assertEqual(calls, ['destroy', 'ok', 'shutdown'])

    def test_observer_cleanup_failure_is_reported_and_shutdown_continues(self):
        calls = []
        errors = []

        class Node:
            def destroy_node(self):
                calls.append('destroy')
                raise RuntimeError('destroy failed')

        failed = cleanup_observer_resources(
            Node(), True,
            lambda: calls.append('ok') or True,
            lambda: calls.append('shutdown'),
            lambda label, exc: errors.append((label, str(exc))))
        self.assertTrue(failed)
        self.assertEqual(calls, ['destroy', 'ok', 'shutdown'])
        self.assertEqual(errors, [('observer node cleanup failed',
                                   'destroy failed')])

    def test_observer_shutdown_failure_is_reported(self):
        errors = []
        failed = cleanup_observer_resources(
            None, True, lambda: True,
            lambda: (_ for _ in ()).throw(RuntimeError('shutdown failed')),
            lambda label, exc: errors.append((label, str(exc))))
        self.assertTrue(failed)
        self.assertEqual(errors, [('ROS 2 shutdown failed',
                                   'shutdown failed')])

    def test_observer_cleanup_skips_uninitialized_context(self):
        calls = []
        failed = cleanup_observer_resources(
            None, False,
            lambda: calls.append('ok'),
            lambda: calls.append('shutdown'),
            lambda label, exc: self.fail('unexpected cleanup error'))
        self.assertFalse(failed)
        self.assertEqual(calls, [])

    def test_observer_main_does_not_report_success_after_cleanup_failure(self):
        observer_path = os.path.join(ROOT, 'tests', 'test_drink.py')
        with open(observer_path, encoding='utf-8') as stream:
            source = stream.read()
        main_start = source.index('def main():')
        main_end = source.index("\n\nif __name__ == '__main__':", main_start)
        main_body = source[main_start:main_end]
        self.assertIn('cleanup_observer_resources(', main_body)
        self.assertIn('if cleanup_failed and result == 0:', main_body)
        self.assertIn('return 3', main_body)
        self.assertNotIn('node.destroy_node()', main_body)
        self.assertNotIn('rclpy.shutdown()', main_body)


    def test_place_held_reports_fresh_object_residual_after_reach_failure(self):
        motion_path = os.path.join(
            ROOT, 'vla_ws', 'src', 'vla_follow', 'vla_follow',
            'drink_motion.py')
        with open(motion_path, encoding='utf-8') as stream:
            source = stream.read()
        start = source.index('def place_held(')
        end = source.index('def put_down(', start)
        body = source[start:end]
        self.assertIn('Tk_after = object_pose(node, model)', body)
        self.assertIn('res_after = pos_err_mm(Tk_after, T_obj_goal)', body)
        self.assertIn('ang_after = axis_angle_deg(Tk_after, T_obj_goal)', body)
        self.assertIn('return False, res_after', body)
        self.assertIn('palm_target = dT @ Tp', body)
        self.assertIn('Tp_after = wrist(node, side)', body)
        self.assertIn('palm_res_after = pos_err_mm(Tp_after, palm_target)', body)
        self.assertIn('how, reach_res', body)
        self.assertIn('物体目标=%s 实测=%s 残差=%.1fmm/%.2f°; q=%s', body)
        self.assertIn('q_after = q_meas(node, side)', body)


    def test_fk_truth_diagnostics_distinguish_world_and_model_relative_links(self):
        from types import SimpleNamespace

        base = (1.1, -0.4, 0.35)
        q = np.asarray(K.Q_REST, float)
        T_robot = K.base_pose(*base)
        T_fk = K.fk_world('L', q, base)

        def pose_message(T):
            # Build a pose-shaped object without ROS, Gazebo or SciPy imports.
            R = T[:3, :3]
            trace = float(np.trace(R))
            if trace > 0.0:
                scale = math.sqrt(trace + 1.0) * 2.0
                qw = 0.25 * scale
                qx = (R[2, 1] - R[1, 2]) / scale
                qy = (R[0, 2] - R[2, 0]) / scale
                qz = (R[1, 0] - R[0, 1]) / scale
            elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
                scale = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
                qw = (R[2, 1] - R[1, 2]) / scale
                qx = 0.25 * scale
                qy = (R[0, 1] + R[1, 0]) / scale
                qz = (R[0, 2] + R[2, 0]) / scale
            elif R[1, 1] > R[2, 2]:
                scale = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
                qw = (R[0, 2] - R[2, 0]) / scale
                qx = (R[0, 1] + R[1, 0]) / scale
                qy = 0.25 * scale
                qz = (R[1, 2] + R[2, 1]) / scale
            else:
                scale = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
                qw = (R[1, 0] - R[0, 1]) / scale
                qx = (R[0, 2] + R[2, 0]) / scale
                qy = (R[1, 2] + R[2, 1]) / scale
                qz = 0.25 * scale
            return SimpleNamespace(
                position=SimpleNamespace(x=T[0, 3], y=T[1, 3], z=T[2, 3]),
                orientation=SimpleNamespace(x=qx, y=qy, z=qz, w=qw))

        robot_pose = pose_message(T_robot)
        world_link = pose_message(T_fk)
        node = SimpleNamespace(
            base=base, links={'robot': robot_pose, 'L_palm': world_link})
        world_diag = M.fk_truth_diagnostics(node, 'L', q)
        self.assertLess(world_diag['base_truth_xy_mm'], 1e-6)
        self.assertLess(abs(world_diag['base_truth_yaw_deg']), 1e-6)
        self.assertLess(world_diag['fk_to_palm_raw_mm'], 1e-6)
        self.assertGreater(world_diag['fk_to_palm_model_mm'], 100.0)

        local_link = np.linalg.inv(T_robot) @ T_fk
        node.links['L_palm'] = pose_message(local_link)
        local_diag = M.fk_truth_diagnostics(node, 'L', q)
        self.assertGreater(local_diag['fk_to_palm_raw_mm'], 100.0)
        self.assertLess(local_diag['fk_to_palm_model_mm'], 1e-6)

    def test_reach_trace_records_ik_fk_and_base_truth_diagnostics(self):
        motion_path = os.path.join(
            ROOT, 'vla_ws', 'src', 'vla_follow', 'vla_follow',
            'drink_motion.py')
        with open(motion_path, encoding='utf-8') as stream:
            source = stream.read()
        start = source.index('def reach(')
        end = source.index('def sim_now(', start)
        body = source[start:end]
        self.assertIn('base_at_ik = _base_pose(node)', body)
        self.assertIn("'odom_twist': ([round(float(v), 5) for v in node.odom_twist]", body)
        task_path = os.path.join(
            ROOT, 'vla_ws', 'src', 'vla_follow', 'vla_follow', 'drink_task.py')
        with open(task_path, encoding='utf-8') as stream:
            task_source = stream.read()
        self.assertIn('self.odom_twist = (float(twist.linear.x)', task_source)
        self.assertIn('self.odom_twist_t = time.monotonic()', task_source)
        self.assertIn('self.js_velocity = {}', task_source)
        self.assertIn("velocity.update(zip(msg.name, getattr(msg, 'velocity', ())))", task_source)
        self.assertIn("if p.name == 'robot':", task_source)
        self.assertIn('self.truth_base_twist = (', task_source)
        self.assertIn("'truth_base_twist': (", body)
        self.assertIn("'wheel_joint_velocity': {", body)
        self.assertIn("result['truth_base_twist']", source)
        self.assertIn("result['wheel_joint_velocity']", source)
        self.assertIn('T_ik_fk = K.fk_world(side, q, base_at_ik)', body)
        self.assertIn("'ik_fk_mm': round(ik_fk_mm, 2)", body)
        self.assertIn("'ik_fk_deg': round(ik_fk_deg, 3)", body)
        self.assertIn('truth_diag = (fk_truth_diagnostics(node, side, q_final)', body)
        self.assertIn('fk_truth=%s', body)

    def test_table_collision_reports_world_geometry_for_capsule_hit(self):
        class Node:
            js = {}
            base = (0.0, 0.0, 0.0)

        box_lo = np.array([0.0, 0.0, 0.0])
        box_hi = np.array([1.0, 1.0, 1.0])
        p0 = np.array([-0.2, 0.5, 0.5])
        p1 = np.array([0.2, 0.5, 0.5])
        palm = (np.array([3.0, 3.0, 3.0]), np.eye(3),
                np.array([0.01, 0.01, 0.01]))
        with patch.object(M, 'q_meas', return_value=np.zeros(K.NJ)), \
                patch.object(M, '_table_collision_boxes',
                             return_value=[('table_top', box_lo, box_hi)]), \
                patch.object(M, '_arm_geometry_world',
                             return_value=([('forearm', p0, p1, 0.03)], palm)):
            clear, hit = M.arm_path_table_clear(
                Node(), 'R', [np.zeros(K.NJ)])

        self.assertFalse(clear)
        self.assertEqual(hit['segment'], 'forearm')
        self.assertEqual(hit['box'], 'table_top')
        self.assertAlmostEqual(hit['check_radius_m'], 0.04)
        np.testing.assert_allclose(hit['segment_start_m'], p0)
        np.testing.assert_allclose(hit['segment_end_m'], p1)
        np.testing.assert_allclose(hit['box_min_m'], box_lo)
        np.testing.assert_allclose(hit['box_max_m'], box_hi)
        np.testing.assert_allclose(hit['expanded_box_min_m'],
                                   box_lo - 0.04)
        np.testing.assert_allclose(hit['expanded_box_max_m'],
                                   box_hi + 0.04)


    def test_reach_table_collision_reports_measured_residual_and_path(self):
        motion_path = os.path.join(
            ROOT, 'vla_ws', 'src', 'vla_follow', 'vla_follow',
            'drink_motion.py')
        with open(motion_path, encoding='utf-8') as stream:
            source = stream.read()
        start = source.index('def reach(')
        end = source.index('def sim_now(', start)
        body = source[start:end]
        self.assertIn('T_current = wrist(node, side)', body)
        self.assertIn('res = pos_err_mm(T_current, T_world)', body)
        self.assertIn("return False, res, 'table-collision'", body)
        self.assertIn('current_xyz=%s target_xyz=%s residual=%.1fmm', body)
        self.assertIn('q_now=%s q_target=%s', body)
        self.assertIn('first_how = None', body)
        self.assertIn('first_how = how', body)
        self.assertIn("'servo_ok': bool(ok_move)", body)
        self.assertIn("'joint_err_deg': round(joint_err_deg, 2)", body)
        self.assertIn('first_ik=%s last_ik=%s trace=%s', body)
        self.assertIn('q_final=%s q_target=%s', body)


    def test_present_uses_strict_servo_tolerance_for_held_cup(self):
        motion_path = os.path.join(
            ROOT, 'vla_ws', 'src', 'vla_follow', 'vla_follow',
            'drink_motion.py')
        with open(motion_path, encoding='utf-8') as stream:
            motion = stream.read()
        start = motion.index('def place_held(')
        end = motion.index('def put_down(', start)
        place_body = motion[start:end]
        self.assertIn('reach_tol_deg=None', place_body)
        self.assertIn('tol_deg=reach_tol_deg', place_body)
        self.assertIn('reach_passes=None, check_table=False', place_body)
        self.assertIn('check_table=check_table', place_body)
        self.assertIn('passes=(len(TOL_SCHED) if reach_passes is None', place_body)
        prepare_start = motion.index('def prepare_present(')
        prepare_end = motion.index('def cup_present_target(', prepare_start)
        self.assertIn('check_table=True', motion[prepare_start:prepare_end])

        task_path = os.path.join(
            ROOT, 'vla_ws', 'src', 'vla_follow', 'vla_follow',
            'drink_task.py')
        with open(task_path, encoding='utf-8') as stream:
            task = stream.read()
        start = task.index("ok_present, present_mm = M.place_held(")
        end = task.index("ok_follow, follow_mm", start)
        present_call = task[start:end]
        self.assertIn('reach_tol_mm=1.5', present_call)
        self.assertIn('reach_tol_deg=0.08', present_call)
        self.assertIn('reach_passes=8', present_call)
        self.assertIn('check_table=True', present_call)
        reach_start = motion.index('def reach(')
        reach_end = motion.index('def sim_now(', reach_start)
        reach_body = motion[reach_start:reach_end]
        self.assertIn('preflight_only=False', reach_body)
        self.assertIn('if preflight_only and not check_table:', reach_body)
        preflight_return = reach_body.index('if preflight_only:')
        servo_move = reach_body.index(
            'ok_move, joint_err = node.arm[side].move(')
        self.assertLess(preflight_return, servo_move)

        preflight_l = task.index(
            "ok_preflight_l, preflight_l_mm, preflight_l_how = M.reach(")
        preflight_r = task.index(
            "ok_preflight_r, preflight_r_mm, preflight_r_how = M.reach(")
        present_move = task.index(
            "ok_present, present_mm = M.place_held(", preflight_r)
        self.assertLess(preflight_l, preflight_r)
        self.assertLess(preflight_r, present_move)
        preflight_calls = task[preflight_l:present_move]
        self.assertEqual(preflight_calls.count('preflight_only=True'), 2)
        self.assertEqual(preflight_calls.count('check_table=True'), 2)
        self.assertIn("FAIL_PRESENT_PREFLIGHT", preflight_calls)

        follow_start = task.index("ok_follow, follow_mm = M.follow_hand(")
        follow_end = task.index("res['present']", follow_start)
        self.assertIn('check_table=True', task[follow_start:follow_end])

        follow_fn = motion.index('def follow_hand(')
        follow_end_fn = motion.index('def hand_gap(', follow_fn)
        self.assertIn('check_table=False', motion[follow_fn:follow_end_fn])
        self.assertIn('check_table=check_table', motion[follow_fn:follow_end_fn])


    def test_reach_tries_only_collision_clear_human_valid_elbow_fallback(self):
        motion_path = os.path.join(
            ROOT, 'vla_ws', 'src', 'vla_follow', 'vla_follow',
            'drink_motion.py')
        with open(motion_path, encoding='utf-8') as stream:
            source = stream.read()
        start = source.index('def reach(')
        end = source.index('def sim_now(', start)
        body = source[start:end]
        self.assertIn("if not _clear and _mode == 'human':", body)
        self.assertIn("elbow='out'", body)
        self.assertIn('alt_bad = K.violation_list(side, q_alt)', body)
        self.assertIn('alt_clear, alt_hit = _path_clear(q_alt)', body)
        self.assertIn('if alt_clear:', body)
        self.assertIn("return False, res, 'table-collision'", body)
        self.assertIn('不拆掉碰撞门控', body)


    def test_reach_searches_human_valid_collision_free_seed_candidates(self):
        motion_path = os.path.join(
            ROOT, 'vla_ws', 'src', 'vla_follow', 'vla_follow',
            'drink_motion.py')
        with open(motion_path, encoding='utf-8') as stream:
            source = stream.read()
        start = source.index('def reach(')
        end = source.index('def sim_now(', start)
        body = source[start:end]
        self.assertIn('K.HUMAN_SEEDS + K.HUMAN_SEEDS_EXTENDED', body)
        self.assertIn('_seed_q = K.human_seed_joints(_seed)', body)
        self.assertIn('base_at_ik, _seed_q, elbow=None', body)
        self.assertIn('refine=False', body)
        self.assertIn('K.ik_world(', body)
        self.assertIn('float(_e_seed) > 4e-3', body)
        self.assertIn('K.violation_list(side, _q_seed)', body)
        self.assertIn('K.human_penalty(side, _q_seed)', body)
        self.assertIn('_seed_clear, _seed_hit = _path_clear(_q_seed)', body)
        self.assertIn('if _seed_clear:', body)
        self.assertIn('human-seed-search-no-clear-path', body)
        self.assertIn('if not _clear:', body)
        self.assertIn("return False, res, 'table-collision'", body)


    def test_world_fk_uses_fresh_gazebo_base_snapshot_and_rejects_stale(self):
        import types
        node = types.SimpleNamespace(
            base=(1.0, 2.0, 0.1),
            truth_base_pose_snapshot=((1.2, 2.3, 0.4), 100.0),
            _truth_base=False, _have_odom=False)
        with patch.object(M.time, 'monotonic', return_value=100.1):
            base = M._base_pose(node)
        self.assertEqual(base, (1.2, 2.3, 0.4))
        self.assertTrue(node._truth_base)
        self.assertTrue(node._have_odom)

        node.truth_base_pose_snapshot = ((9.0, 9.0, 1.0), 90.0)
        with patch.object(M.time, 'monotonic', return_value=100.1):
            stale_base = M._base_pose(node)
        self.assertEqual(stale_base, (1.2, 2.3, 0.4))

    def test_reach_and_wrist_refresh_world_base_from_truth_snapshot(self):
        motion_path = os.path.join(
            ROOT, 'vla_ws', 'src', 'vla_follow', 'vla_follow',
            'drink_motion.py')
        with open(motion_path, encoding='utf-8') as stream:
            source = stream.read()
        wrist_start = source.index('def wrist(')
        wrist_end = source.index('def object_pose(', wrist_start)
        self.assertIn('_base_pose(node)', source[wrist_start:wrist_end])
        reach_start = source.index('def reach(')
        reach_end = source.index('def sim_now(', reach_start)
        self.assertIn('base_at_ik = _base_pose(node)',
                      source[reach_start:reach_end])

    def test_reach_collision_preflight_uses_one_frozen_ik_snapshot(self):
        motion_path = os.path.join(
            ROOT, 'vla_ws', 'src', 'vla_follow', 'vla_follow',
            'drink_motion.py')
        with open(motion_path, encoding='utf-8') as stream:
            source = stream.read()
        path_start = source.index('def arm_path_table_clear(')
        path_end = source.index('def reach(', path_start)
        path_body = source[path_start:path_end]
        self.assertIn('q_start=None', path_body)
        self.assertIn('q0 = (q_meas(node, side) if q_start is None', path_body)
        reach_start = source.index('def reach(')
        reach_end = source.index('def sim_now(', reach_start)
        reach_body = source[reach_start:reach_end]
        self.assertIn('base=base_at_ik', reach_body)
        self.assertIn('q_start=q_now', reach_body)
        self.assertIn("side, T_world, base_at_ik, q_now, elbow='out'", reach_body)
        self.assertIn('side, T_world, base_at_ik, _seed_q, elbow=None,', reach_body)
        self.assertIn('base_at_ik=%s truth_base_at_ik=%s', reach_body)
        self.assertIn('joint_age_at_ik_s=%s truth_age_at_ik_s=%s', reach_body)

    def test_human_posture_seed_expands_to_full_seven_axis_ik_seed(self):
        compact = np.asarray(K.HUMAN_SEEDS[17], float)
        expanded = K.human_seed_joints(compact)
        self.assertEqual(expanded.shape, (K.NJ,))
        np.testing.assert_allclose(expanded[:4], compact)
        np.testing.assert_allclose(expanded[4:], K.Q_REST[4:])
        self.assertTrue(np.all(np.isfinite(expanded)))
        self.assertTrue(np.all(expanded >= K.LIM_LO))
        self.assertTrue(np.all(expanded <= K.LIM_HI))

    def test_extended_human_seed_grid_is_bounded_and_valid(self):
        self.assertEqual(len(K.HUMAN_SEEDS_EXTENDED), 72)
        for seed in K.HUMAN_SEEDS_EXTENDED:
            expanded = K.human_seed_joints(seed)
            self.assertEqual(expanded.shape, (K.NJ,))
            self.assertTrue(np.all(np.isfinite(expanded)))
            self.assertTrue(np.all(expanded >= K.LIM_LO))
            self.assertTrue(np.all(expanded <= K.LIM_HI))


    def test_human_posture_seed_expansion_rejects_invalid_input(self):
        invalid = (
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.0, 0.0, 0.0),
            (0.0, 0.0, float('nan'), 0.0),
        )
        for seed in invalid:
            with self.subTest(seed=seed):
                with self.assertRaises(ValueError):
                    K.human_seed_joints(seed)

    def test_ik_refine_reduces_fk_pose_error_without_exceeding_joint_limits(self):
        q_goal = np.asarray(K.Q_REST, float).copy()
        q_goal += np.array([0.12, -0.08, 0.10, -0.10, 0.08, -0.08, 0.10])
        q_seed = q_goal + np.array(
            [0.04, -0.03, 0.035, -0.04, 0.03, -0.025, 0.03])
        target = K.fk('L', q_goal)
        before = float(np.linalg.norm(
            K.pose_err(K.fk('L', q_seed), target)))
        q_refined, after = K.ik_refine('L', target, q_seed)
        self.assertLess(after, before)
        self.assertLessEqual(after, before * 0.25)
        self.assertTrue(np.all(q_refined >= K.LIM_LO - 1e-9))
        self.assertTrue(np.all(q_refined <= K.LIM_HI + 1e-9))
        self.assertAlmostEqual(
            after,
            float(np.linalg.norm(K.pose_err(K.fk('L', q_refined), target))),
            places=9)

    def test_ik_best_polishes_kdl_or_dls_candidate_when_residual_is_large(self):
        kin_path = os.path.join(
            ROOT, 'vla_ws', 'src', 'vla_follow', 'vla_follow', 'arm_kin.py')
        with open(kin_path, encoding='utf-8') as stream:
            source = stream.read()
        start = source.index('def ik_refine(')
        end = source.index('def reach_ok(', start)
        refine_body = source[start:end]
        self.assertIn('for alpha in (1.0, 0.5, 0.25, 0.125, 0.0625):',
                      refine_body)
        self.assertIn('q_try = np.clip(q + alpha * dq, LIM_LO, LIM_HI)',
                      refine_body)
        self.assertIn('if err_try < err -', refine_body)
        self.assertIn('if refine and e > 5e-4:', source)
        self.assertIn('q_refined, e_refined = ik_refine(side, T_tgt, q)',
                      source)


    def test_follow_handback_starts_with_alignment_and_handover_logs_true_geometry(self):
        follower_path = os.path.join(
            ROOT, 'vla_ws', 'src', 'vla_follow', 'vla_follow', 'follower.py')
        with open(follower_path, encoding='utf-8') as stream:
            follower = stream.read()
        start = follower.index('def _cmd_cb(')
        end = follower.index('# ---------- Nav2 搜索段接口', start)
        follow_cb = follower[start:end]
        self.assertIn("self.state = 'FOLLOW'", follow_cb)
        self.assertIn('self._aligning = True', follow_cb)
        self.assertIn('self._align_edge_t = 0.0', follow_cb)
        self.assertIn('self._align_dir = 0.0', follow_cb)
        self.assertLess(follow_cb.index("self.state = 'FOLLOW'"),
                        follow_cb.index('self._aligning = True'))

        task_path = os.path.join(
            ROOT, 'vla_ws', 'src', 'vla_follow', 'vla_follow', 'drink_task.py')
        with open(task_path, encoding='utf-8') as stream:
            task = stream.read()
        self.assertIn('HANDOVER_GEOM world_m', task)
        self.assertIn('cup_minus_mid_mm', task)
        self.assertIn('person_arm_l_joint', task)
        self.assertIn('M.person_hand_poses(self)', task)



    def test_reach_trace_captures_drink_and_muxed_base_commands(self):
        motion_path = os.path.join(
            ROOT, 'vla_ws', 'src', 'vla_follow', 'vla_follow', 'drink_motion.py')
        with open(motion_path, encoding='utf-8') as stream:
            motion = stream.read()
        for field in ("'drink_cmd_vel':", "'drink_cmd_age_s':",
                      "'robot_cmd_vel':", "'robot_cmd_age_s':"):
            self.assertIn(field, motion)

        task_path = os.path.join(
            ROOT, 'vla_ws', 'src', 'vla_follow', 'vla_follow', 'drink_task.py')
        with open(task_path, encoding='utf-8') as stream:
            task = stream.read()
        self.assertIn("self.create_subscription(Twist, '/drink/cmd_vel'", task)
        self.assertIn("self.create_subscription(Twist, '/robot/cmd_vel'", task)
        self.assertIn('def _drink_cmd_cb(self, msg):', task)
        self.assertIn('def _robot_cmd_cb(self, msg):', task)


    def test_robot_diff_drive_wheels_have_bounded_effort_limits(self):
        world_path = os.path.join(
            ROOT, 'vla_ws', 'src', 'vla_follow', 'worlds', 'work_area.sdf')
        root = ET.parse(world_path).getroot()
        robot = root.find(".//world/model[@name='robot']")
        self.assertIsNotNone(robot)
        for name in ('left_wheel_joint', 'right_wheel_joint'):
            joint = robot.find("./joint[@name='%s']" % name)
            self.assertIsNotNone(joint)
            effort = joint.findtext('./axis/limit/effort')
            self.assertIsNotNone(effort, '%s must define wheel torque authority' % name)
            self.assertAlmostEqual(float(effort), 5.0, places=6)


    def test_put_back_failure_logs_release_and_cup_state_diagnostics(self):
        task_path = os.path.join(
            ROOT, 'vla_ws', 'src', 'vla_follow', 'vla_follow', 'drink_task.py')
        with open(task_path, encoding='utf-8') as stream:
            task = stream.read()
        start = task.index("ok_d, res_d = M.put_down_two(")
        end = task.index("# 物体已安全释放", start)
        failure_path = task[start:end]
        self.assertIn("_cup_fail_pose = M.object_pose(self, 'cup')", failure_path)
        self.assertIn("_holding_fail = list(self.carry.holding())", failure_path)
        self.assertIn("putback_info=%s", failure_path)
        self.assertIn("cup_home_mm=%.1f", failure_path)
        self.assertIn("_pb", failure_path)


    def test_put_down_distinguishes_release_from_withdraw_failure(self):
        motion_path = os.path.join(
            ROOT, 'vla_ws', 'src', 'vla_follow', 'vla_follow',
            'drink_motion.py')
        with open(motion_path, encoding='utf-8') as stream:
            motion = stream.read()
        start = motion.index('def put_down(')
        end = motion.index('# ---------------------------------------------------------------- 双手捧', start)
        body = motion[start:end]
        release = body.index('release_ok = node.carry.release(model)')
        confirmed = body.index(
            "info.update(release_confirmed=True, object_state='released')")
        withdraw = body.index('ok_back, res_back, how_back = reach(')
        self.assertLess(release, confirmed)
        self.assertLess(confirmed, withdraw)
        self.assertIn("object_state='still_attached'", body)
        self.assertIn("failure_stage='arm_withdraw_after_release'", body)
        self.assertIn('已确认释放并落桌，但撤手未到位', body)
        # Both pre-release placement and post-open verification retain the
        # strict reach tolerance while allowing additional refinement passes.
        self.assertEqual(body.count('reach_passes=8'), 2)

        prepare = motion[motion.index('def prepare_present('):]
        self.assertIn("kettle_putdown.get('release_confirmed')", prepare)
        self.assertIn("'kettle_retract_failed_after_release'", prepare)
        self.assertNotIn(
            '水壶未确认落稳，保持抓持并中止后续动作', prepare)

    def test_put_down_retries_table_lip_with_collision_checked_lift_waypoint(self):
        motion_path = os.path.join(
            ROOT, 'vla_ws', 'src', 'vla_follow', 'vla_follow',
            'drink_motion.py')
        with open(motion_path, encoding='utf-8') as stream:
            motion = stream.read()
        start = motion.index('def put_down(')
        end = motion.index('# ---------------------------------------------------------------- 双手捧', start)
        body = motion[start:end]
        first = body.index('ok_back, res_back, how_back = reach(')
        gate = body.index("if not ok_back and how_back == 'table-collision':")
        lift = body.index('T_stage[2, 3] += 0.12', gate)
        stage_reach = body.index(
            'node, side, T_stage, tol_mm=8.0, check_table=True', lift)
        retry = body.index('node, side, T_back, tol_mm=8.0, check_table=True',
                           stage_reach)
        self.assertLess(first, gate)
        self.assertLess(gate, lift)
        self.assertLess(lift, stage_reach)
        self.assertLess(stage_reach, retry)
        self.assertIn('if ok_stage:', body[stage_reach:retry])
        self.assertIn('withdraw_stage', body)
        # Both the lift and retry keep the original full table collision gate.
        self.assertEqual(body.count('check_table=True'), 3)

    def test_reach_enforces_ik_fk_four_mm_before_table_checked_motion(self):
        motion_path = os.path.join(
            ROOT, 'vla_ws', 'src', 'vla_follow', 'vla_follow',
            'drink_motion.py')
        with open(motion_path, encoding='utf-8') as stream:
            motion = stream.read()
        start = motion.index('def reach(')
        end = motion.index('def sim_now(', start)
        body = motion[start:end]
        gate = body.index('if ik_fk_mm > 4.0:')
        self.assertIn('float(_e_seed) > 4e-3', body[gate:])
        self.assertIn('_seed_fk_mm > 4.0', body[gate:])
        self.assertIn('K.violation_list(side, _q_seed)', body[gate:])
        self.assertIn('_seed_clear, _seed_hit = _path_clear(_q_seed)', body[gate:])
        self.assertIn("return False, ik_fk_mm, 'ik-residual'", body[gate:])
        self.assertIn('if _ik_seed_candidates:', body[gate:])

    def test_reach_servo_failure_keeps_joint_and_base_trace(self):
        motion_path = os.path.join(
            ROOT, 'vla_ws', 'src', 'vla_follow', 'vla_follow',
            'drink_motion.py')
        with open(motion_path, encoding='utf-8') as stream:
            motion = stream.read()
        start = motion.index('def reach(')
        end = motion.index('def sim_now(', start)
        body = motion[start:end]
        self.assertIn('q_failed = q_meas(node, side)', body)
        self.assertIn('joint_delta_deg=%s trace=%s', body)
        self.assertIn('q_target=%s q_measured=%s', body)
        self.assertIn('base_at_ik=%s', body)
        self.assertIn('truth_base_twist_at_ik', body)

    def test_reach_command_telemetry_rejects_stale_cached_zero(self):
        motion_path = os.path.join(
            ROOT, 'vla_ws', 'src', 'vla_follow', 'vla_follow',
            'drink_motion.py')
        with open(motion_path, encoding='utf-8') as stream:
            motion = stream.read()
        self.assertIn('def _command_evidence(snapshot):', motion)
        self.assertIn('_fresh = _age <= 0.5', motion)
        self.assertIn("'drink_cmd_fresh': _drink_fresh", motion)
        self.assertIn("'robot_cmd_fresh': _robot_fresh", motion)
        self.assertIn("'drink_cmd_vel': _drink_vel", motion)
        self.assertIn("'robot_cmd_vel': _robot_vel", motion)

    def test_joint_and_truth_pose_snapshots_are_timestamped(self):
        task_path = os.path.join(
            ROOT, 'vla_ws', 'src', 'vla_follow', 'vla_follow',
            'drink_task.py')
        motion_path = os.path.join(
            ROOT, 'vla_ws', 'src', 'vla_follow', 'vla_follow',
            'drink_motion.py')
        with open(task_path, encoding='utf-8') as stream:
            task = stream.read()
        with open(motion_path, encoding='utf-8') as stream:
            motion = stream.read()
        self.assertIn('self.js_sample_t = 0.0', task)
        self.assertIn('self.js_sample_t = time.monotonic()', task)
        self.assertIn('self.truth_base_pose_snapshot = None', task)
        self.assertIn('self.truth_base_pose_snapshot = (pose, sample_t)', task)
        self.assertIn("'joint_state_age_s':", motion)
        self.assertIn("'truth_base_pose_age_s':", motion)
        self.assertIn("'truth_base_pose_xyz_yaw':", motion)
        self.assertIn('truth_snapshot_at_ik = getattr(', motion)
        self.assertIn("'joint_state_age_at_ik_s':", motion)
        self.assertIn("'truth_base_pose_age_at_ik_s':", motion)
        self.assertIn("'truth_base_pose_at_ik':", motion)
        self.assertIn("'base_truth_xy_mm_at_ik':", motion)
        self.assertIn("'base_truth_yaw_deg_at_ik':", motion)

    def test_reach_settles_sim_after_servo_before_world_pose_measurement(self):
        motion_path = os.path.join(
            ROOT, 'vla_ws', 'src', 'vla_follow', 'vla_follow',
            'drink_motion.py')
        with open(motion_path, encoding='utf-8') as stream:
            motion = stream.read()
        start = motion.index('def reach(')
        end = motion.index('def sim_now(', start)
        body = motion[start:end]
        move = body.index('ok_move, joint_err = node.arm[side].move(')
        settle = body.index('settle_sim(node, 0.20)', move)
        measure = body.index('T_after_move = wrist(node, side)', settle)
        self.assertLess(move, settle)
        self.assertLess(settle, measure)
        grasp_start = motion.index('def grasp(')
        grasp_end = motion.index('def held_palm_target(', grasp_start)
        self.assertIn('tol_mm=GRASP_TOL_MM, passes=8',
                      motion[grasp_start:grasp_end])
        self.assertIn('if not ok or res > GRASP_TOL_MM * 3.0',
                      motion[grasp_start:grasp_end])

    def test_put_down_two_uses_preflighted_convergent_waypoint_sweep(self):
        motion_path = os.path.join(
            ROOT, 'vla_ws', 'src', 'vla_follow', 'vla_follow',
            'drink_motion.py')
        with open(motion_path, encoding='utf-8') as stream:
            motion = stream.read()
        start = motion.index('def put_down_two(')
        end = motion.index('# ------------------------------------------------------------------ 收臂/抬臂',
                           start)
        body = motion[start:end]
        first_attempt = body.index("how_r == 'table-collision'")
        sweep = body.index('for _candidate in putback_route_candidates(T_right_goal):',
                           first_attempt)
        preflight = body.index('preflight_table_route(', sweep)
        execute_waypoint = body.index("node, 'R', _route_waypoint", preflight)
        retry_final = body.index("node, 'R', T_right_goal", execute_waypoint)
        self.assertLess(first_attempt, sweep)
        self.assertLess(sweep, preflight)
        self.assertLess(preflight, execute_waypoint)
        self.assertLess(execute_waypoint, retry_final)
        self.assertIn('PUTBACK_ROUTE_TRIAL', body)
        self.assertIn('PUTBACK_ROUTE_ACCEPTED', body)
        self.assertIn("_route_elbows = (['out', elbow] if elbow == 'human'", body)
        preflight_start = motion.index('def preflight_table_route(')
        preflight_end = motion.index('def put_down_two(', preflight_start)
        preflight_body = motion[preflight_start:preflight_end]
        self.assertIn('K.HUMAN_SEEDS + K.HUMAN_SEEDS_EXTENDED', preflight_body)
        self.assertIn('if not clear:', preflight_body)
        self.assertIn('alt_clear, _alt_hit = arm_path_table_clear(', preflight_body)
        reach_start = motion.index('def reach(')
        reach_end = motion.index('def place_held(', reach_start)
        reach_body = motion[reach_start:reach_end]
        self.assertIn('K.HUMAN_SEEDS + K.HUMAN_SEEDS_EXTENDED', reach_body)
        self.assertIn('elbow=_route_elbow)', body)
        self.assertIn('elbow=_route_elbow, check_table=True)', body)
        self.assertIn("'preflight=%s detail=%s'", body)
        self.assertIn("'params=%s residual=%.1fmm'", body)
        self.assertIn("'PUTBACK_ROUTE_ACCEPTED step=%d trial=%d elbow=%s '", body)
        self.assertIn('PUTBACK_ROUTE_EXHAUSTED', body)
        self.assertIn('PUTBACK_ROUTE_SUMMARY', body)
        self.assertIn('_route_reasons', body)
        self.assertIn("release_block_reason='right_arm_path_failed'", body)
        self.assertIn("node.carry.release(model)", body)
        right_failure = body.index("if not ok_r:", retry_final)
        release = body.index("node.carry.release(model)")
        self.assertLess(right_failure, release)


    def test_drink_observer_timeout_exceeds_drive_to_hard_wall_limit(self):
        observer_path = os.path.join(ROOT, 'tests', 'test_drink.py')
        with open(observer_path, encoding='utf-8') as stream:
            source = stream.read()
        import ast
        tree = ast.parse(source)
        timeout = next(
            node.value for node in tree.body
            if isinstance(node, ast.Assign)
            and any(isinstance(target, ast.Name)
                    and target.id == 'TOTAL_TIMEOUT'
                    for target in node.targets))
        self.assertIsInstance(timeout, ast.Constant)
        self.assertGreater(timeout.value, 18.0 * 30.0)
        self.assertLessEqual(timeout.value, 600.0)

    def test_command_snapshots_start_unknown_not_as_zero(self):
        task_path = os.path.join(
            ROOT, 'vla_ws', 'src', 'vla_follow', 'vla_follow',
            'drink_task.py')
        with open(task_path, encoding='utf-8') as stream:
            task = stream.read()
        self.assertIn('self._drink_cmd_snapshot = None', task)
        self.assertIn('self._robot_cmd_snapshot = None', task)

    def test_base_motion_snapshot_requires_fresh_stationary_wheels_and_truth(self):
        from vla_follow import drink_motion as M
        self.assertTrue(M.base_motion_snapshot_ok(
            {'left_wheel_joint': 0.01, 'right_wheel_joint': -0.02},
            0.10, (0.002, -0.001, 0.004), 0.12))
        self.assertFalse(M.base_motion_snapshot_ok(
            {'left_wheel_joint': -0.986, 'right_wheel_joint': 0.0},
            0.10, (0.0, 0.0, 0.0), 0.10))
        self.assertFalse(M.base_motion_snapshot_ok(
            {'left_wheel_joint': 0.0, 'right_wheel_joint': 0.0},
            0.10, (0.0, 0.0, 0.02), 0.10))
        self.assertFalse(M.base_motion_snapshot_ok(
            {'left_wheel_joint': 0.0, 'right_wheel_joint': 0.0},
            0.30, (0.0, 0.0, 0.0), 0.10))
        self.assertFalse(M.base_motion_snapshot_ok(
            {'left_wheel_joint': 0.0},
            0.10, (0.0, 0.0, 0.0), 0.10))

    def test_place_robot_stops_and_verifies_base_before_accepting_teleport(self):
        motion_path = os.path.join(
            ROOT, 'vla_ws', 'src', 'vla_follow', 'vla_follow',
            'drink_motion.py')
        with open(motion_path, encoding='utf-8') as stream:
            source = stream.read()
        helper_start = source.index('def base_motion_snapshot_ok(')
        helper_end = source.index('def place_robot(', helper_start)
        helper = source[helper_start:helper_end]
        self.assertIn("node.drive(0.0, 0.0)", helper)
        self.assertIn("'left_wheel_joint', 'right_wheel_joint'", helper)
        self.assertIn('truth_twist_age_s', helper)
        self.assertIn('stable_since', helper)
        start = source.index('def place_robot(')
        end = source.index('def yaw_err_of(', start)
        body = source[start:end]
        self.assertLess(body.index('if not wait_base_stopped(node):'),
                        body.index("node.carry.place('robot', T_goal)"))
        settle_at = body.index('settle_sim(node, settle)')
        post_wait_at = body.index(
            'if not wait_base_stopped(node):', settle_at)
        self.assertLess(body.index('if not wait_base_stopped(node):'),
                        settle_at)
        self.assertLess(settle_at, post_wait_at)
        self.assertLess(post_wait_at,
                        body.index("T_now = object_pose(node, 'robot')"))


    def test_person_hand_joint_angles_recovered_from_parent_child_truth(self):
        from vla_follow import drink_motion as M
        parent = np.eye(4)
        parent[:3, :3] = M.K.rot_z(0.37)
        left = parent.copy()
        right = parent.copy()
        left[:3, :3] = parent[:3, :3] @ M.K.rot_z(0.42)
        right[:3, :3] = parent[:3, :3] @ M.K.rot_z(-0.31)
        angles = M.person_hand_joint_angles_from_truth(parent, left, right)
        self.assertIsNotNone(angles)
        self.assertAlmostEqual(angles[0], 0.42, places=6)
        self.assertAlmostEqual(angles[1], -0.31, places=6)
        invalid = right.copy()
        invalid[:3, :3] = parent[:3, :3] @ M.K.rot_x(0.1)
        self.assertIsNone(
            M.person_hand_joint_angles_from_truth(parent, left, invalid))
        self.assertIsNone(
            M.person_hand_joint_angles_from_truth(parent, left, None))

    def test_person_hand_midpoint_world_uses_link_truth_not_joint_state_topic(self):
        motion_path = os.path.join(
            ROOT, 'vla_ws', 'src', 'vla_follow', 'vla_follow',
            'drink_motion.py')
        with open(motion_path, encoding='utf-8') as stream:
            source = stream.read()
        start = source.index('def person_hand_midpoint_world(')
        end = source.index('def person_hand_midpoint_offset(', start)
        body = source[start:end]
        self.assertIn('person_hand_joint_angles_from_truth(', body)
        self.assertIn("link='person_arms'", body)
        self.assertIn("link='person_arm_l'", body)
        self.assertIn("link='person_arm_r'", body)
        self.assertNotIn("getattr(node, 'js'", body)



    def test_reach_collision_failure_reports_endpoint_collision_separately(self):
        motion_path = os.path.join(
            ROOT, 'vla_ws', 'src', 'vla_follow', 'vla_follow',
            'drink_motion.py')
        with open(motion_path, encoding='utf-8') as stream:
            source = stream.read()
        start = source.index('def reach(')
        end = source.index('\ndef ', start + 5)
        body = source[start:end]
        self.assertIn('_endpoint_clear, _endpoint_hit = arm_path_table_clear(', body)
        self.assertIn('q_start=q', body)
        self.assertIn('endpoint_clear=%s endpoint_hit=%s', body)
        self.assertIn('diagnostic only', body)


    def test_follow_hand_trace_captures_wheel_and_command_high_water(self):
        task_path = os.path.join(
            ROOT, 'vla_ws', 'src', 'vla_follow', 'vla_follow',
            'drink_task.py')
        with open(task_path, encoding='utf-8') as stream:
            source = stream.read()
        self.assertIn("wheel_names = ('left_wheel_joint', 'right_wheel_joint')", source)
        self.assertIn("wheel_max_abs_rad_s", source)
        self.assertIn("wheel_max_abs_effort_nm", source)
        self.assertIn("def _record_follow_cmd_trace(self, channel, msg):", source)
        self.assertIn("drink_cmd_peak_angular", source)
        self.assertIn("robot_cmd_peak_angular", source)

    def test_putback_route_sweep_is_bounded_and_varies_only_approach_waypoint(self):
        motion_path = os.path.join(
            ROOT, 'vla_ws', 'src', 'vla_follow', 'vla_follow',
            'drink_motion.py')
        with open(motion_path, encoding='utf-8') as stream:
            source = stream.read()
        self.assertIn('def putback_route_candidates(T_goal):', source)
        self.assertIn('def preflight_table_route(', source)
        self.assertIn('max_ik_error_mm=4.0', source)
        self.assertIn('arm_path_table_clear(', source)
        self.assertIn('PUTBACK_ROUTE_TRIAL', source)
        self.assertIn('PUTBACK_ROUTE_ACCEPTED', source)
        self.assertIn('PUTBACK_ROUTE_EXHAUSTED', source)
        self.assertIn('保持双手持杯并阻止 release', source)
        self.assertIn('putback_route_candidates(T_right_goal)', source)
        self.assertIn('[_route_waypoint, T_right_goal]', source)
        # The final support target remains the original rigidly-derived target;
        # only the intermediate approach waypoint is parameterized.
        self.assertIn("node, 'R', T_right_goal, tol_mm=6.0", source)
        self.assertIn("_route_elbows = (['out', elbow] if elbow == 'human'", source)
        self.assertIn("elbow=_route_elbow, check_table=True", source)
        self.assertIn('max_ik_error_mm', source)
        goal = np.eye(4, dtype=float)
        goal[:3, 3] = [2.2, 0.36, 0.773]
        original = goal.copy()
        candidates = M.putback_route_candidates(goal)
        self.assertEqual(len(candidates), 27)
        self.assertTrue(np.array_equal(goal, original))
        self.assertEqual(
            [(c['height_m'], c['dx_m'], c['dy_m']) for c in candidates[:9]],
            [(0.20, 0.0, 0.0), (0.20, 0.0, -0.04),
             (0.20, 0.0, 0.04), (0.20, -0.04, 0.0),
             (0.20, 0.04, 0.0), (0.20, -0.04, -0.04),
             (0.20, -0.04, 0.04), (0.20, 0.04, -0.04),
             (0.20, 0.04, 0.04)])



    def test_reach_seed_search_counts_intrinsic_endpoint_collisions(self):
        motion_path = os.path.join(
            ROOT, 'vla_ws', 'src', 'vla_follow', 'vla_follow',
            'drink_motion.py')
        with open(motion_path, encoding='utf-8') as stream:
            source = stream.read()
        start = source.index('def reach(')
        end = source.index('def sim_now(', start)
        body = source[start:end]
        self.assertIn('_endpoint_seed_clear = 0', body)
        self.assertIn('_endpoint_seed_ok, _endpoint_seed_hit = (', body)
        self.assertIn('endpoint_clear_seeds=%d', body)
        self.assertIn('q_start=_q_seed', body)
        self.assertIn('endpoint_hits=%s', body)


    def test_handover_logs_cup_and_palms_before_and_after_person_close(self):
        task_path = os.path.join(
            ROOT, 'vla_ws', 'src', 'vla_follow', 'vla_follow',
            'drink_task.py')
        with open(task_path, encoding='utf-8') as stream:
            source = stream.read()
        self.assertIn('HANDOVER_PRE_CLOSE cup_xyz=%s left_xyz=%s right_xyz=%s gap_mm=%s', source)
        self.assertIn('HANDOVER_POST_CLOSE cup_xyz=%s cup_drift_mm=%s gap_mm=%s', source)
        self.assertIn('_close_drift_mm', source)
        self.assertLess(source.index('HANDOVER_PRE_CLOSE'), source.index('ok_person_grip = M.person_hands'))
        self.assertLess(source.index('ok_person_grip = M.person_hands'), source.index('HANDOVER_POST_CLOSE'))


    def test_grasp_approach_and_final_pose_require_table_path_gate(self):
        motion_path = os.path.join(
            ROOT, 'vla_ws', 'src', 'vla_follow', 'vla_follow',
            'drink_motion.py')
        with open(motion_path, encoding='utf-8') as stream:
            source = stream.read()
        start = source.index('def grasp(')
        end = source.index('def held_palm_target(', start)
        body = source[start:end]
        self.assertIn(
            "node, side, T_pre, tol_mm=8.0, check_table=True", body)
        self.assertIn(
            "node, side, T_grasp, tol_mm=GRASP_TOL_MM, passes=8,\n        check_table=True",
            body)
        self.assertLess(body.index('ok_pre, res_pre, how_pre = reach('),
                        body.index('if not ok_pre:'))
        self.assertLess(body.index('if not ok_pre:'),
                        body.index('ok, res, how = reach('))
        self.assertLess(body.index('if not ok or res > GRASP_TOL_MM * 3.0:'),
                        body.index('node.carry.grab(model, side)'))



    def test_present_follow_cup_drift_fails_closed_before_person_close(self):
        task_path = os.path.join(
            ROOT, 'vla_ws', 'src', 'vla_follow', 'vla_follow',
            'drink_task.py')
        with open(task_path, encoding='utf-8') as stream:
            source = stream.read()
        self.assertIn('PRESENT_POST_FOLLOW_TARGET', source)
        self.assertIn("self._set_state('FAIL_PRESENT_FOLLOW_DRIFT')", source)
        self.assertIn('保持机器人持杯，不请求人手合拢、不交权', source)
        guard = source.index("self._set_state('FAIL_PRESENT_FOLLOW_DRIFT')")
        close = source.index('ok_person_grip = M.person_hands')
        self.assertLess(source.index('_post_follow_pos_mm > 6.0'), guard)
        self.assertLess(guard, close)
        self.assertIn('_post_follow_axis_deg > 0.5', source)



    def test_drive_wheels_have_bounded_zero_command_damping(self):
        world_path = os.path.join(
            ROOT, 'vla_ws', 'src', 'vla_follow', 'worlds', 'work_area.sdf')
        with open(world_path, encoding='utf-8') as stream:
            world = stream.read()
        for joint in ('left_wheel_joint', 'right_wheel_joint'):
            start = world.index('<joint name="%s"' % joint)
            end = world.index('</joint>', start)
            block = world[start:end]
            self.assertIn('<effort>5.0</effort>', block)
            self.assertIn('<dynamics><damping>0.5</damping></dynamics>', block)


    def test_follow_hand_trace_records_full_chassis_rpy_and_rejects_invalid_quaternion(self):
        task_path = os.path.join(ROOT, 'vla_ws', 'src', 'vla_follow', 'vla_follow', 'drink_task.py')
        with open(task_path, encoding='utf-8') as stream:
            source = stream.read()
        self.assertIn('def quaternion_rpy(x, y, z, w):', source)
        self.assertIn('if norm < 1e-9: return None', source)
        self.assertIn("'truth_rpy':", source)
        self.assertIn("'max_abs_roll_delta_deg': 0.0", source)
        self.assertIn("'max_abs_pitch_delta_deg': 0.0", source)
        self.assertIn("trace['end_rpy'] = list(rpy)", source)

if __name__ == '__main__':
    unittest.main(verbosity=2)

    def test_handover_geometry_failure_log_formats_pair_without_tuple_unpack(self):
        task_path = os.path.join(
            ROOT, 'vla_ws', 'src', 'vla_follow', 'vla_follow',
            'drink_task.py')
        with open(task_path, encoding='utf-8') as stream:
            source = stream.read()
        start = source.index('_pre_hc = M.hands_to_cup_mm(self)')
        end = source.index('            # 保存世界系手心中点', start)
        body = source[start:end]
        self.assertIn("'不可用' if _pre_hc is None else", body)
        self.assertIn("'L=%.1fmm, R=%.1fmm' % (", body)
        self.assertIn('% _pre_hc_text)', body)
        self.assertNotIn('% (None if _pre_hc is None', body)
        self.assertIn("self._set_state('FAIL_HANDOVER_GEOMETRY')", body)
        self.assertIn('保持机器人持杯，不交权', body)

