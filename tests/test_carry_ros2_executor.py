#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""真实 ROS 2 executor 下 CarryBus ACK 投递与 request_id 隔离测试。

运行环境：ROS 2 Jazzy。由单一 MultiThreadedExecutor 所有权驱动节点；
测试线程只发布 ACK 并观察事务结果，不调用 spin_once/spin_for。
"""
import os
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

from vla_follow.carry import ACK_TOPIC, CarryBus  # noqa: E402


class TestCarryRos2Executor(unittest.TestCase):
    def test_executor_delivers_concurrent_ids_and_rejects_late_ack(self):
        rclpy.init()
        node = Node("carry_ack_executor_integration_test")
        bus = CarryBus(node)
        ack_pub = node.create_publisher(String, ACK_TOPIC, 10)
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()
        try:
            discovery_deadline = time.monotonic() + 5.0
            while ack_pub.get_subscription_count() == 0:
                self.assertLess(time.monotonic(), discovery_deadline,
                                "CarryBus ACK subscription was not discovered")
                time.sleep(0.01)

            request_ids = (9101, 9102)
            with bus._ack_lock:
                bus.pending_requests.update(request_ids)

            for request_id in request_ids:
                ack_pub.publish(String(
                    data="ok joint cup water_joint %d" % request_id))

            delivery_deadline = time.monotonic() + 5.0
            while True:
                with bus._ack_lock:
                    received = {
                        request_id: bus.ack_by_request.get(request_id)
                        for request_id in request_ids
                    }
                if all(received.values()):
                    break
                self.assertLess(time.monotonic(), delivery_deadline,
                                "ROS 2 executor did not deliver both ACKs")
                time.sleep(0.01)

            self.assertEqual(
                received,
                {
                    9101: "ok joint cup water_joint 9101",
                    9102: "ok joint cup water_joint 9102",
                })

            # Expire 9101, register a different waiter, then deliver its late ACK.
            with bus._ack_lock:
                bus.pending_requests.discard(9101)
                bus.ack_by_request.pop(9101, None)
                bus.pending_requests.add(9103)
            ack_pub.publish(String(
                data="ok joint cup water_joint 9101"))

            late_deadline = time.monotonic() + 1.0
            while time.monotonic() < late_deadline:
                with bus._ack_lock:
                    self.assertNotIn(9101, bus.ack_by_request)
                    self.assertNotIn(9103, bus.ack_by_request)
                time.sleep(0.01)
        finally:
            executor.shutdown(timeout_sec=2.0)
            spin_thread.join(timeout=2.0)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    unittest.main()
