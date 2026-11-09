#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""CarryBus ↔ ros_gz_bridge ↔ Gazebo GraspCarry transaction test.

Run after starting the GraspCarry test world and a parameter_bridge instance
whose ACK output is routed to /drink/carry/ack_raw. The relay intentionally
drops the first ACK so CarryBus must retry and the plugin must replay its
completed transaction result.
"""
import os
import signal
import subprocess
import sys
import threading
import time
import unittest

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "vla_ws", "src", "vla_follow"))

from vla_follow.carry import ACK_TOPIC, CARRY_TOPIC, CarryBus  # noqa: E402


RAW_ACK_TOPIC = ACK_TOPIC + "_raw"


class AckRelay(Node):
    def __init__(self):
        super().__init__("grasp_carry_ack_fault_injector")
        self.publisher = self.create_publisher(String, ACK_TOPIC, 20)
        self.dropped_first = threading.Event()
        self.forwarded = []
        self.drop_next = threading.Event()
        self.dropped_next = threading.Event()
        self.subscription = self.create_subscription(
            String, RAW_ACK_TOPIC, self._on_ack, 20)

    def _on_ack(self, message):
        if not self.dropped_first.is_set():
            self.dropped_first.set()
            self.get_logger().warning(
                "intentionally dropped first GraspCarry terminal ACK")
            return
        if self.drop_next.is_set():
            self.drop_next.clear()
            self.dropped_next.set()
            self.get_logger().warning(
                "intentionally dropped ACK during bridge restart test")
            return
        self.forwarded.append(message.data)
        self.publisher.publish(message)


class TestCarryGazeboBridge(unittest.TestCase):
    def test_retry_replays_ack_across_ros_gz_bridge(self):
        rclpy.init()
        node = Node("carry_gazebo_bridge_test")
        relay = AckRelay()
        bus = CarryBus(node, executor_managed=True)
        executor = MultiThreadedExecutor(num_threads=3)
        executor.add_node(node)
        executor.add_node(relay)
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()
        try:
            deadline = time.monotonic() + 20.0
            while time.monotonic() < deadline:
                if (bus.pub.get_subscription_count() > 0 and
                        relay.count_publishers(RAW_ACK_TOPIC) >= 1 and
                        relay.count_subscribers(RAW_ACK_TOPIC) >= 1):
                    break
                time.sleep(0.1)
            else:
                self.fail("ROS bridge topics did not become ready")

            command = "place cup 0 0 0.5 0 0 0 1"
            ok = bus._await(
                lambda request_id: bus._send(
                    "%s %d" % (command, request_id)),
                lambda ack, request_id: ack ==
                "ok place cup %d" % request_id,
                timeout=8.0)

            self.assertTrue(relay.dropped_first.wait(2.0),
                            "first plugin ACK was not observed/dropped")
            self.assertTrue(
                ok, "CarryBus did not recover after the first ACK was dropped")
            self.assertGreaterEqual(len(relay.forwarded), 1)
            self.assertEqual(len(set(relay.forwarded)), 1,
                             "replayed ACK payload changed across retries")
            self.assertTrue(relay.forwarded[0].startswith("ok place cup "))

            # Two concurrent requests share the same CarryBus and executor.
            # Each must consume only the ACK carrying its own request ID.
            results = {}
            errors = []

            def transact(value):
                try:
                    result = bus._await(
                        lambda request_id: bus._send(
                            "joint cup water_joint %.6f %d" %
                            (value, request_id)),
                        lambda ack, request_id: ack ==
                        "ok joint cup water_joint %d" % request_id,
                        timeout=5.0)
                    results[value] = result
                except Exception as exc:
                    errors.append(exc)

            workers = [
                threading.Thread(target=transact, args=(0.006,)),
                threading.Thread(target=transact, args=(0.018,)),
            ]
            for worker in workers:
                worker.start()
            for worker in workers:
                worker.join(timeout=8.0)
                self.assertFalse(worker.is_alive(),
                                 "CarryBus bridge transaction hung")
            self.assertFalse(errors, "transaction exceptions: %r" % errors)
            self.assertEqual(results, {0.006: True, 0.018: True})
            self.assertFalse(bus.pending_requests)
            self.assertFalse(bus.ack_by_request)
            self.assertFalse(bus._ack_events)

            # Exercise a real parameter_bridge process restart while a CarryBus
            # waiter is active. The plugin has accepted the command, but the
            # relay drops its terminal ACK; after restart, the same request ID
            # must be retried and the plugin's completed-ACK cache must replay it.
            bridge_pid = int(os.environ["CARRY_BRIDGE_PID"])
            bridge_config = os.environ["CARRY_BRIDGE_CONFIG"]
            relay.dropped_next.clear()
            relay.drop_next.set()
            restart_result = {}

            def restart_transaction():
                restart_result["ok"] = bus._await(
                    lambda request_id: bus._send(
                        "place cup 0 0 0.5 0 0 0 1 %d" % request_id),
                    lambda ack, request_id: ack ==
                    "ok place cup %d" % request_id,
                    timeout=12.0)

            restart_worker = threading.Thread(
                target=restart_transaction, daemon=True)
            restart_worker.start()
            self.assertTrue(
                relay.dropped_next.wait(5.0),
                "plugin ACK was not observed before bridge restart")

            # Stop the actual bridge process, not merely the ROS subscriber.
            os.kill(bridge_pid, signal.SIGTERM)
            deadline = time.monotonic() + 5.0
            while time.monotonic() < deadline:
                try:
                    os.kill(bridge_pid, 0)
                except ProcessLookupError:
                    break
                time.sleep(0.05)
            else:
                os.kill(bridge_pid, signal.SIGKILL)
                self.fail("ros_gz_bridge did not stop after SIGTERM")

            bridge_log = "/tmp/grasp-carry-bridge-restarted.log"
            with open(bridge_log, "w", encoding="utf-8") as stream:
                restarted_bridge = subprocess.Popen(
                    ["ros2", "run", "ros_gz_bridge", "parameter_bridge",
                     "--ros-args", "-p",
                     "config_file:=" + bridge_config],
                    stdout=stream, stderr=subprocess.STDOUT,
                    start_new_session=True)
            try:
                ready_deadline = time.monotonic() + 15.0
                while time.monotonic() < ready_deadline:
                    if restarted_bridge.poll() is not None:
                        with open(bridge_log, encoding="utf-8") as log:
                            self.fail("restarted bridge exited: " + log.read())
                    with open(bridge_log, encoding="utf-8") as log:
                        bridge_output = log.read()
                    if ("Creating GZ->ROS Bridge" in bridge_output and
                            "Creating ROS->GZ Bridge" in bridge_output and
                            relay.count_publishers(RAW_ACK_TOPIC) >= 1 and
                            bus.pub.get_subscription_count() > 0):
                        break
                    time.sleep(0.1)
                else:
                    with open(bridge_log, encoding="utf-8") as log:
                        self.fail("restarted bridge not ready: " + log.read())
                restart_worker.join(timeout=8.0)
                self.assertFalse(
                    restart_worker.is_alive(),
                    "CarryBus waiter hung across bridge restart")
                self.assertTrue(
                    restart_result.get("ok"),
                    "same-request retry did not recover after bridge restart")
            finally:
                restarted_bridge.terminate()
                try:
                    restarted_bridge.wait(timeout=3.0)
                except subprocess.TimeoutExpired:
                    restarted_bridge.kill()
                    restarted_bridge.wait(timeout=3.0)

            self.assertFalse(bus.pending_requests)
            self.assertFalse(bus.ack_by_request)
            self.assertFalse(bus._ack_events)
        finally:
            bus.cancel_waiters()
            executor.shutdown(timeout_sec=3.0)
            spin_thread.join(timeout=3.0)
            executor.remove_node(node)
            executor.remove_node(relay)
            node.destroy_node()
            relay.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    unittest.main(verbosity=2)
