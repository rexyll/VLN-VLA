#!/usr/bin/env python3
"""Unit tests for persistent Gazebo Transport chassis-lock publishers."""
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "vla_ws" / "src" / "vla_follow"))

from gz.msgs10.stringmsg_pb2 import StringMsg
from vla_follow.gazebo_ground_lock import GazeboGroundLock


class FakePublisher:
    def __init__(self):
        self.messages = []
        self.on_publish = None

    def publish(self, message):
        self.messages.append(message)
        if self.on_publish:
            self.on_publish(message)
        return True


class FakeTransportNode:
    def __init__(self):
        self.publishers = {}
        self.topic = None
        self.callback = None

    def subscribe(self, msg_type, topic, callback):
        self.topic = topic
        self.callback = callback
        return True

    def advertise(self, topic, msg_type):
        publisher = FakePublisher()
        self.publishers[topic] = publisher
        return publisher


class GazeboGroundLockPublisherTests(unittest.TestCase):
    def make_lock(self):
        node = FakeTransportNode()
        lock = GazeboGroundLock(node, "test_world")
        return lock, node

    def test_advertises_persistent_attach_and_detach_publishers(self):
        lock, node = self.make_lock()
        self.assertIn(lock.attach_topic, node.publishers)
        self.assertIn(lock.detach_topic, node.publishers)

    def test_retries_until_fresh_state_event(self):
        lock, node = self.make_lock()
        publisher = node.publishers[lock.detach_topic]

        def publish_side_effect(_message):
            if len(publisher.messages) == 3:
                node.callback(StringMsg(data="detached"))

        publisher.on_publish = publish_side_effect
        self.assertTrue(lock.command("detached", timeout_s=1.0))
        self.assertEqual(lock.last_command_attempts, 3)
        self.assertEqual(lock.state, "detached")
        self.assertEqual(len(publisher.messages), 3)

    def test_force_requires_fresh_event_even_if_cached_state_matches(self):
        lock, node = self.make_lock()
        node.callback(StringMsg(data="attached"))
        publisher = node.publishers[lock.attach_topic]
        publisher.on_publish = lambda _msg: node.callback(StringMsg(data="attached"))
        self.assertTrue(lock.command("attached", timeout_s=0.5, force=True))
        self.assertEqual(len(publisher.messages), 1)

    def test_cached_matching_state_is_idempotent_without_publish(self):
        lock, node = self.make_lock()
        node.callback(StringMsg(data="detached"))
        self.assertTrue(lock.command("detached", timeout_s=0.5))
        self.assertEqual(node.publishers[lock.detach_topic].messages, [])

    def test_timeout_reports_multiple_publish_attempts(self):
        lock, node = self.make_lock()
        self.assertFalse(lock.command("detached", timeout_s=0.7))
        self.assertGreaterEqual(lock.last_command_attempts, 2)
        self.assertEqual(len(node.publishers[lock.detach_topic].messages),
                         lock.last_command_attempts)


if __name__ == "__main__":
    unittest.main()
