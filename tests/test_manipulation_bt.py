import os
import unittest

from vla_follow.manipulation_bt import (
    Action, CandidateBuffer, Checkpoint, FailureCandidate, RecoverySupervisor,
    Retry, Selector, Sequence, Status, WorldSnapshot,
)


class BehaviorTreeTests(unittest.TestCase):
    def test_sequence_resumes_running_child(self):
        calls = []
        def first(_ctx):
            calls.append("first")
            return Status.SUCCESS
        states = iter((Status.RUNNING, Status.SUCCESS))
        def second(_ctx):
            calls.append("second")
            return next(states)
        tree = Sequence(Action(first), Action(second))
        self.assertEqual(tree.tick({}), Status.RUNNING)
        self.assertEqual(tree.tick({}), Status.SUCCESS)
        self.assertEqual(calls, ["first", "second", "second"])

    def test_selector_uses_fallback_after_local_recovery_fails(self):
        calls = []
        tree = Selector(
            Action(lambda _ctx: (calls.append("local"), Status.FAILURE)[1]),
            Action(lambda _ctx: (calls.append("safe_stop"), Status.SUCCESS)[1]))
        self.assertEqual(tree.tick({}), Status.SUCCESS)
        self.assertEqual(calls, ["local", "safe_stop"])

    def test_retry_budget_is_bounded_and_running_does_not_consume_it(self):
        states = iter((Status.RUNNING, Status.FAILURE, Status.FAILURE))
        retry = Retry(Action(lambda _ctx: next(states)), max_attempts=2)
        self.assertEqual(retry.tick({}), Status.RUNNING)
        self.assertEqual(retry.attempts, 0)
        self.assertEqual(retry.tick({}), Status.RUNNING)
        self.assertEqual(retry.attempts, 1)
        self.assertEqual(retry.tick({}), Status.FAILURE)
        self.assertEqual(retry.attempts, 0)


class CandidateBufferTests(unittest.TestCase):
    def candidate(self, n, score, valid=False):
        return FailureCandidate(
            candidate_id=str(n), position_residual_mm=score,
            rotation_residual_deg=0.1, score=score, hard_valid=valid,
            target_xyz=(float(n), 0.0, 0.0), target_rpy=(0.0, 0.0, 0.0))

    def test_keeps_best_five_distinct_candidates(self):
        buffer = CandidateBuffer(5)
        for n in range(8):
            buffer.add(self.candidate(n, float(8 - n), valid=(n == 7)))
        self.assertEqual(len(buffer), 5)
        self.assertEqual([c.candidate_id for c in buffer.ranked()],
                         ["7", "6", "5", "4", "3"])
        self.assertEqual([c.candidate_id for c in buffer.executable()], ["7"])

    def test_deduplicates_and_never_promotes_invalid_candidate(self):
        buffer = CandidateBuffer(3)
        item = self.candidate(1, 5.0, valid=False)
        buffer.add(item)
        buffer.add(self.candidate(1, 4.0, valid=False))
        self.assertEqual(len(buffer), 1)
        self.assertEqual(buffer.ranked()[0].score, 4.0)
        self.assertEqual(buffer.executable(), [])

    def test_hard_valid_duplicate_replaces_lower_scored_invalid_record(self):
        buffer = CandidateBuffer(3)
        invalid = self.candidate(1, 1.0, valid=False)
        valid = self.candidate(1, 9.0, valid=True)
        buffer.add(invalid)
        buffer.add(valid)
        self.assertEqual(len(buffer), 1)
        self.assertEqual(buffer.executable(), [valid])


    def test_capacity_is_strictly_three_to_five(self):
        for capacity in (2, 6):
            with self.assertRaises(ValueError):
                CandidateBuffer(capacity)


class BacktrackGuardTests(unittest.TestCase):
    def setUp(self):
        self.checkpoint = Checkpoint(
            checkpoint_id="C-end", action_name="C", scene_token="scene-A",
            base_pose=(1.0, 2.0, 0.2), object_state="kettle_released",
            irreversible_epoch=3, replayable=True,
            safe_recovery_pose_verified=True)
        self.current = WorldSnapshot(
            scene_token="scene-A", base_pose=(1.001, 2.001, 0.205),
            object_state="kettle_released", irreversible_epoch=3)

    def test_matching_checkpoint_allows_one_bounded_backtrack(self):
        supervisor = RecoverySupervisor(max_backtracks=1)
        self.assertEqual(supervisor.authorize_backtrack(
            self.checkpoint, self.current), (True, "checkpoint_revalidated"))
        self.assertEqual(supervisor.authorize_backtrack(
            self.checkpoint, self.current),
            (False, "backtrack_budget_exhausted"))

    def test_irreversible_event_blocks_replay(self):
        supervisor = RecoverySupervisor()
        changed = WorldSnapshot(**{**self.current.__dict__,
                                   "irreversible_epoch": 4})
        self.assertEqual(supervisor.can_backtrack(self.checkpoint, changed),
                         (False, "irreversible_event_boundary_crossed"))

    def test_unknown_or_changed_world_state_fails_closed(self):
        supervisor = RecoverySupervisor()
        unknown = WorldSnapshot(**{**self.current.__dict__, "state_known": False})
        self.assertEqual(supervisor.can_backtrack(self.checkpoint, unknown)[0],
                         False)
        changed_object = WorldSnapshot(**{**self.current.__dict__,
                                          "object_state": "kettle_held"})
        self.assertEqual(supervisor.can_backtrack(
            self.checkpoint, changed_object),
            (False, "object_state_changed"))

    def test_nonreplayable_or_unverified_safe_pose_blocks_replay(self):
        supervisor = RecoverySupervisor()
        nonreplayable = Checkpoint(**{**self.checkpoint.__dict__,
                                      "replayable": False})
        unsafe = Checkpoint(**{**self.checkpoint.__dict__,
                               "safe_recovery_pose_verified": False})
        self.assertEqual(supervisor.can_backtrack(nonreplayable, self.current)[1],
                         "upstream_action_not_replayable")
        self.assertEqual(supervisor.can_backtrack(unsafe, self.current)[1],
                         "safe_recovery_pose_not_verified")



class RuntimeIntegrationTests(unittest.TestCase):
    def test_post_release_withdraw_uses_local_selector_and_forbids_upstream_replay(self):
        root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        motion_path = os.path.join(
            root, "vla_ws", "src", "vla_follow", "vla_follow",
            "drink_motion.py")
        with open(motion_path, encoding="utf-8") as stream:
            source = stream.read()
        start = source.index("def put_down(")
        end = source.index("# ---------------------------------------------------------------- 双手捧", start)
        body = source[start:end]
        self.assertIn("BT.Selector(*_bt_children)", body)
        self.assertIn("BT.Sequence(", body)
        self.assertIn("'upstream_replay_allowed': False", body)
        self.assertIn("'release_boundary': 'irreversible_object_release'", body)
        self.assertLess(body.index("release_ok = node.carry.release(model)"),
                        body.index("BT.Selector(*_bt_children)"))
        self.assertIn("check_table=True", body)
        self.assertIn("tol_mm=8.0", body)


if __name__ == "__main__":
    unittest.main(verbosity=2)
