#!/usr/bin/env python3
"""Unit tests for retrying Gazebo DetachableJoint state transitions."""
import sys
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "vla_ws" / "src" / "vla_follow"))

from gz.msgs10.stringmsg_pb2 import StringMsg
from vla_follow.gazebo_ground_lock import GazeboGroundLock


class FakeTransportNode:
    def subscribe(self, msg_type, topic, callback):
        self.topic = topic
        self.callback = callback
        return True


class GazeboGroundLockRetryTests(unittest.TestCase):
    def make_lock(self):
        node = FakeTransportNode()
        lock = GazeboGroundLock(node, "test_world")
        return lock, node

    @patch("vla_follow.gazebo_ground_lock.subprocess.run")
    def test_retries_until_fresh_state_event(self, publish):
        lock, node = self.make_lock()
        calls = []

        def publish_side_effect(*args, **kwargs):
            calls.append(args[0])
            if len(calls) == 3:
                node.callback(StringMsg(data="detached"))
            return SimpleNamespace(returncode=0, stdout="")

        publish.side_effect = publish_side_effect
        self.assertTrue(lock.command("detached", timeout_s=1.0))
        self.assertEqual(lock.last_command_attempts, 3)
        self.assertEqual(lock.state, "detached")
        self.assertTrue(all("chassis_lock/detach" in x for x in calls))

    @patch("vla_follow.gazebo_ground_lock.subprocess.run")
    def test_force_requires_fresh_event_even_if_cached_state_matches(self, publish):
        lock, node = self.make_lock()
        node.callback(StringMsg(data="attached"))
        publish.side_effect = lambda *a, **k: (
            node.callback(StringMsg(data="attached"))
            or SimpleNamespace(returncode=0, stdout="")
        )
        self.assertTrue(lock.command("attached", timeout_s=0.5, force=True))
        publish.assert_called_once()

    @patch("vla_follow.gazebo_ground_lock.subprocess.run")
    def test_cached_matching_state_is_idempotent_without_publish(self, publish):
        lock, node = self.make_lock()
        node.callback(StringMsg(data="detached"))
        self.assertTrue(lock.command("detached", timeout_s=0.5))
        publish.assert_not_called()

    @patch("vla_follow.gazebo_ground_lock.subprocess.run")
    def test_timeout_reports_multiple_attempts(self, publish):
        publish.return_value = SimpleNamespace(returncode=0, stdout="")
        lock, _ = self.make_lock()
        self.assertFalse(lock.command("detached", timeout_s=0.7))
        self.assertGreaterEqual(lock.last_command_attempts, 2)


if __name__ == "__main__":
    unittest.main()
