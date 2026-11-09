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
from types import SimpleNamespace

import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "vla_ws", "src", "vla_follow"))

from vla_follow.carry import ACK_TOPIC, CarryBus  # noqa: E402
from vla_follow.drink_task import (  # noqa: E402
    CmdBus, DrinkTask, _shutdown_node)



class TestCarryRos2Executor(unittest.TestCase):
    def test_cancel_waiter_serializes_with_initial_emit(self):
        rclpy.init()
        node = Node("carry_initial_emit_cancel_race_test")
        bus = CarryBus(node, executor_managed=True)
        emit_entered = threading.Event()
        cancel_attempted = threading.Event()
        allow_emit_return = threading.Event()
        emitted = []
        result = {}

        def emit(request_id):
            emitted.append(request_id)
            emit_entered.set()
            self.assertTrue(allow_emit_return.wait(timeout=2.0),
                            "initial emit barrier was not released")

        waiter = threading.Thread(target=lambda: result.setdefault(
            "ok", bus._await(emit, lambda *_args: True, 5.0)))
        waiter.start()
        try:
            self.assertTrue(emit_entered.wait(timeout=2.0),
                            "initial transaction emit did not start")

            def cancel():
                cancel_attempted.set()
                bus.cancel_waiters()

            canceller = threading.Thread(target=cancel)
            canceller.start()
            self.assertTrue(cancel_attempted.wait(timeout=1.0))
            # The cancellation thread must wait for the in-progress publish's
            # lock, then prevent every subsequent retry.
            allow_emit_return.set()
            waiter.join(timeout=2.0)
            canceller.join(timeout=2.0)
            self.assertFalse(waiter.is_alive(), "transaction waiter did not exit")
            self.assertFalse(canceller.is_alive(), "cancellation did not finish")
            self.assertEqual(result, {"ok": False})
            self.assertEqual(len(emitted), 1,
                             "cancellation allowed a post-cancel emit")
            with bus._ack_lock:
                self.assertFalse(bus.pending_requests)
                self.assertFalse(bus.ack_by_request)
                self.assertFalse(bus._ack_events)
        finally:
            bus.cancel_waiters()
            node.destroy_node()
            rclpy.shutdown()

    def test_follower_status_and_timestamp_are_one_snapshot(self):
        state = SimpleNamespace(_follower_status_snapshot=({}, 0.0))
        msg = SimpleNamespace(data='{"state":"FOLLOW","track":7,"lost":false}')
        DrinkTask._follower_status_cb(state, msg)
        payload, received_at = state._follower_status_snapshot
        self.assertEqual(payload["state"], "FOLLOW")
        self.assertEqual(payload["track"], 7)
        self.assertGreater(received_at, 0.0)
        previous = state._follower_status_snapshot
        DrinkTask._follower_status_cb(
            state, SimpleNamespace(data="{malformed"))
        self.assertIs(state._follower_status_snapshot, previous)

    def test_drink_task_state_updates_publish_outside_task_lock(self):
        lock = threading.RLock()
        published = []

        class Publisher:
            def publish(self, message):
                # ROS publication must not execute while task state is locked.
                acquired = lock.acquire(blocking=False)
                if not acquired:
                    raise AssertionError("publish called while task lock held")
                lock.release()
                published.append(message.data)

        state = SimpleNamespace(
            _task_lock=lock,
            state="BOOT",
            _busy=False,
            pub_state=Publisher())
        DrinkTask._set_state(state, "IDLE")
        self.assertEqual(published, ['{"state": "IDLE"}'])
        self.assertEqual(DrinkTask._task_status_snapshot(state), ("IDLE", False))
        DrinkTask._set_state(state, "IDLE")
        self.assertEqual(len(published), 1,
                         "unchanged state should not emit a duplicate update")

    def test_drink_task_status_snapshot_never_tears_state_and_busy(self):
        state = SimpleNamespace(
            _task_lock=threading.RLock(),
            state="IDLE",
            _busy=False)
        start = threading.Barrier(3)
        errors = []

        def writer():
            try:
                start.wait(timeout=3.0)
                for _ in range(5000):
                    with state._task_lock:
                        state.state = "RUNNING"
                        state._busy = True
                    with state._task_lock:
                        state.state = "IDLE"
                        state._busy = False
            except Exception as exc:
                errors.append(exc)

        def reader():
            try:
                start.wait(timeout=3.0)
                for _ in range(10000):
                    snapshot = DrinkTask._task_status_snapshot(state)
                    if snapshot not in (("IDLE", False), ("RUNNING", True)):
                        errors.append(AssertionError(
                            "observed torn task status: %r" % (snapshot,)))
                        return
            except Exception as exc:
                errors.append(exc)

        workers = [threading.Thread(target=writer),
                   threading.Thread(target=reader)]
        for worker in workers:
            worker.start()
        start.wait(timeout=3.0)
        for worker in workers:
            worker.join(timeout=5.0)
            self.assertFalse(worker.is_alive(),
                             "task status snapshot worker did not finish")
        self.assertFalse(errors, "task status snapshot race: %r" % errors)

    def test_shutdown_cancels_waiters_before_destroying_node(self):
        events = []

        class FakeCarry:
            def cancel_waiters(self):
                events.append("cancel_waiters")

        class FakeNode:
            carry = FakeCarry()

            def destroy_node(self):
                events.append("destroy_node")

        _shutdown_node(FakeNode())
        self.assertEqual(events, ["cancel_waiters", "destroy_node"])

    def test_shutdown_destroys_node_even_if_waiter_cancel_raises(self):
        events = []

        class FakeCarry:
            def cancel_waiters(self):
                events.append("cancel_waiters")
                raise RuntimeError("cancel failed")

        class FakeNode:
            carry = FakeCarry()

            def destroy_node(self):
                events.append("destroy_node")

        with self.assertRaisesRegex(RuntimeError, "cancel failed"):
            _shutdown_node(FakeNode())
        self.assertEqual(events, ["cancel_waiters", "destroy_node"])

    def test_drink_task_pending_action_claim_is_atomic(self):
        state = SimpleNamespace(
            _task_lock=threading.RLock(),
            _busy=False,
            _want_drink=True,
            _want_home=True,
            _want_hang=False)
        start = threading.Barrier(3)
        claimed = []
        errors = []

        def claim():
            try:
                start.wait(timeout=3.0)
                claimed.append(DrinkTask._take_pending_action(state))
            except Exception as exc:
                errors.append(exc)

        workers = [threading.Thread(target=claim) for _ in range(2)]
        for worker in workers:
            worker.start()
        start.wait(timeout=3.0)
        for worker in workers:
            worker.join(timeout=3.0)
            self.assertFalse(worker.is_alive(),
                             "pending action claim worker did not finish")
        self.assertFalse(errors, "pending action claim errors: %r" % errors)
        self.assertEqual(claimed.count("drink"), 1)
        self.assertEqual(claimed.count(None), 1)
        self.assertTrue(state._busy)
        self.assertFalse(state._want_drink)
        self.assertTrue(state._want_home)
        DrinkTask._finish_pending_action(state)
        self.assertFalse(state._busy)

    def test_drink_task_joint_callback_publishes_coherent_snapshots(self):
        state = SimpleNamespace(js={})
        errors = []
        start = threading.Barrier(3)

        def writer():
            try:
                start.wait(timeout=3.0)
                for value in range(1, 5001):
                    msg = SimpleNamespace(
                        name=("joint_a", "joint_b"),
                        position=(float(value), float(value)))
                    DrinkTask._js_cb(state, msg)
            except Exception as exc:
                errors.append(exc)

        def reader():
            try:
                start.wait(timeout=3.0)
                for _ in range(10000):
                    snapshot = state.js
                    if "joint_a" in snapshot and "joint_b" in snapshot:
                        if snapshot["joint_a"] != snapshot["joint_b"]:
                            errors.append(AssertionError(
                                "joint callback exposed a partial update: %r"
                                % snapshot))
                            return
            except Exception as exc:
                errors.append(exc)

        threads = [threading.Thread(target=writer),
                   threading.Thread(target=reader)]
        for thread in threads:
            thread.start()
        start.wait(timeout=3.0)
        for thread in threads:
            thread.join(timeout=5.0)
            self.assertFalse(thread.is_alive(),
                             "sensor snapshot worker did not finish")
        self.assertFalse(errors, "sensor snapshot race: %r" % errors)
        self.assertEqual(state.js, {"joint_a": 5000.0, "joint_b": 5000.0})

    def test_cmd_bus_atomic_multi_joint_snapshots(self):
        rclpy.init()
        node = Node("cmd_bus_concurrency_test")
        bus = CmdBus(node, names=("joint_a", "joint_b"),
                     init={"joint_a": 0.0, "joint_b": 0.0})
        errors = []
        start = threading.Barrier(3)

        def writer():
            try:
                start.wait(timeout=3.0)
                for value in range(1, 5001):
                    bus.set(("joint_a", "joint_b"), (value, value))
                    if value % 17 == 0:
                        bus.publish_all()
            except Exception as exc:
                errors.append(exc)

        def reader():
            try:
                start.wait(timeout=3.0)
                for _ in range(5000):
                    values = bus.get_many(("joint_a", "joint_b"))
                    if values[0] != values[1]:
                        errors.append(AssertionError(
                            "multi-joint snapshot observed a torn update: %r"
                            % values))
                        return
                    bus.get("joint_a")
            except Exception as exc:
                errors.append(exc)

        threads = [threading.Thread(target=writer),
                   threading.Thread(target=reader)]
        for thread in threads:
            thread.start()
        start.wait(timeout=3.0)
        for thread in threads:
            thread.join(timeout=5.0)
            self.assertFalse(thread.is_alive(),
                             "CmdBus concurrency worker did not finish")
        self.assertFalse(errors, "CmdBus concurrency errors: %r" % errors)
        self.assertEqual(bus.get_many(("joint_a", "joint_b")).tolist(),
                         [5000.0, 5000.0])
        self.assertGreater(bus.count, 0)
        node.destroy_node()
        rclpy.shutdown()

    def test_carry_attachment_state_is_safe_during_updates(self):
        rclpy.init()
        node = Node("carry_attachment_concurrency_test")
        bus = CarryBus(node)
        bus.host_world = lambda *_args: np.eye(4)
        bus.obj_world = lambda _model: np.eye(4)
        # This test isolates attachment-map synchronization from transport ACKs.
        bus._await = lambda *_args: True
        errors = []
        start = threading.Barrier(3)

        def mutate_attachments():
            try:
                start.wait(timeout=3.0)
                for index in range(2000):
                    model = "item_%d" % (index % 8)
                    if not bus.hand_to(model, "person"):
                        errors.append(AssertionError(
                            "hand_to unexpectedly rejected %s" % model))
                    if index % 2 == 0:
                        bus.release(model)
            except Exception as exc:
                errors.append(exc)

        def stream_updates():
            try:
                start.wait(timeout=3.0)
                for _ in range(3000):
                    bus.update()
                    bus.holding()
            except Exception as exc:
                errors.append(exc)

        threads = [threading.Thread(target=mutate_attachments),
                   threading.Thread(target=stream_updates)]
        for thread in threads:
            thread.start()
        start.wait(timeout=3.0)
        for thread in threads:
            thread.join(timeout=8.0)
            self.assertFalse(thread.is_alive(),
                             "CarryBus attachment worker did not finish")
        self.assertFalse(errors, "attachment state race: %r" % errors)
        node.destroy_node()
        rclpy.shutdown()

    def test_executor_delivers_concurrent_ids_and_rejects_late_ack(self):
        rclpy.init()
        node = Node("carry_ack_executor_integration_test")
        bus = CarryBus(node, executor_managed=True)
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

            # First terminal ACK wins on the real DDS subscription as well:
            # identical duplicates are harmless and conflicting terminal
            # outcomes cannot overwrite the first result.
            with bus._ack_lock:
                bus.pending_requests.update((9104, 9105))
            terminal_messages = (
                "ok joint cup water_joint 9104",
                "ok joint cup water_joint 9104",
                "fail joint cup water_joint 9104 queue-full",
                "fail joint cup water_joint 9105 queue-full",
                "fail joint cup water_joint 9105 queue-full",
                "ok joint cup water_joint 9105",
            )
            for data in terminal_messages:
                ack_pub.publish(String(data=data))
            terminal_deadline = time.monotonic() + 5.0
            while True:
                with bus._ack_lock:
                    terminal = {
                        request_id: bus.ack_by_request.get(request_id)
                        for request_id in (9104, 9105)
                    }
                if all(terminal.values()):
                    break
                self.assertLess(time.monotonic(), terminal_deadline,
                                "ROS 2 executor did not deliver terminal ACKs")
                time.sleep(0.01)
            self.assertEqual(terminal, {
                9104: "ok joint cup water_joint 9104",
                9105: "fail joint cup water_joint 9105 queue-full",
            })

            # Exercise the real CarryBus._await() path while the external
            # MultiThreadedExecutor owns callback progress. Two transaction
            # threads wait independently; neither is allowed to spin the node.
            transaction_results = {}
            transaction_ids = {}
            start = threading.Barrier(3)

            both_registered = threading.Event()

            def run_transaction(name):
                start.wait(timeout=3.0)

                def emit(request_id):
                    transaction_ids[name] = request_id
                    if len(transaction_ids) == 2:
                        both_registered.set()

                transaction_results[name] = bus._await(
                    emit,
                    lambda ack, rid: ack == "ok transaction %s %d" % (name, rid),
                    2.0)

            waiters = [
                threading.Thread(target=run_transaction, args=("left",)),
                threading.Thread(target=run_transaction, args=("right",)),
            ]
            for waiter in waiters:
                waiter.start()
            start.wait(timeout=3.0)
            self.assertTrue(
                both_registered.wait(timeout=1.0),
                "both concurrent CarryBus waiters did not register")
            # Publish only after both request IDs are pending. This separates
            # executor delivery from DDS discovery / publisher startup timing
            # and ensures the test actually exercises two overlapping waiters.
            for name in ("left", "right"):
                ack_pub.publish(String(
                    data="ok transaction %s %d" %
                    (name, transaction_ids[name])))
            for waiter in waiters:
                waiter.join(timeout=4.0)
                self.assertFalse(waiter.is_alive(),
                                 "executor-managed transaction waiter hung")
            self.assertEqual(transaction_results, {"left": True, "right": True})
            self.assertNotEqual(transaction_ids["left"], transaction_ids["right"])

            # A timeout must remove both pending state and its wakeup Event.
            timed_out_ids = []
            self.assertFalse(bus._await(
                lambda rid: timed_out_ids.append(rid),
                lambda ack, rid: ack == "ok timeout %d" % rid,
                0.12))
            self.assertGreaterEqual(len(timed_out_ids), 1)
            self.assertEqual(len(set(timed_out_ids)), 1)
            timed_out_id = timed_out_ids[0]
            ack_pub.publish(String(data="ok timeout %d" % timed_out_id))
            time.sleep(0.15)
            with bus._ack_lock:
                self.assertNotIn(timed_out_id, bus.pending_requests)
                self.assertNotIn(timed_out_id, bus.ack_by_request)
                self.assertNotIn(timed_out_id, bus._ack_events)

            # A terminal queue-full ACK is a rejection, not a retryable success.
            queue_full_ids = []
            self.assertFalse(bus._await(
                lambda rid: (
                    queue_full_ids.append(rid),
                    ack_pub.publish(String(
                        data="fail transaction %d queue-full" % rid))),
                lambda ack, rid: ack == "ok transaction %d" % rid,
                1.0))
            self.assertGreaterEqual(len(queue_full_ids), 1)
            self.assertEqual(len(set(queue_full_ids)), 1,
                             "queue-full retransmits changed request_id")

            # ACK consumption wins over a later cancellation, but cancellation
            # must prevent remaining duplicate ACK retransmits. Hold the second
            # emit at a deterministic barrier so cancel_waiters races that path
            # without relying on scheduler timing.
            race_bus = CarryBus(node, executor_managed=True)
            race_ids = []
            retransmit_entered = threading.Event()
            allow_retransmit = threading.Event()
            race_result = {}

            def race_emit(request_id):
                race_ids.append(request_id)
                if len(race_ids) == 1:
                    ack_pub.publish(String(data="ok race %d" % request_id))
                elif len(race_ids) == 2:
                    retransmit_entered.set()
                    self.assertTrue(allow_retransmit.wait(timeout=2.0),
                                    "retransmit barrier was not released")

            race_waiter = threading.Thread(target=lambda: race_result.setdefault(
                "ok", race_bus._await(
                    race_emit,
                    lambda ack, rid: ack == "ok race %d" % rid,
                    3.0)))
            race_waiter.start()
            self.assertTrue(retransmit_entered.wait(timeout=2.0),
                            "successful ACK did not reach retransmit phase")
            cancel_started = threading.Event()

            def cancel_after_ack():
                cancel_started.set()
                race_bus.cancel_waiters()

            race_canceller = threading.Thread(target=cancel_after_ack)
            race_canceller.start()
            self.assertTrue(cancel_started.wait(timeout=1.0))
            allow_retransmit.set()
            race_waiter.join(timeout=2.0)
            race_canceller.join(timeout=2.0)
            self.assertFalse(race_waiter.is_alive(),
                             "ACK/cancel race waiter did not finish")
            self.assertFalse(race_canceller.is_alive(),
                             "ACK/cancel race cancellation did not finish")
            self.assertEqual(race_result, {"ok": True})
            self.assertEqual(len(race_ids), 2,
                             "cancellation allowed post-race duplicate emits")
            with race_bus._ack_lock:
                self.assertNotIn(race_ids[0], race_bus.pending_requests)
                self.assertNotIn(race_ids[0], race_bus.ack_by_request)
                self.assertNotIn(race_ids[0], race_bus._ack_events)

            # Executor owner cancellation must wake a waiter immediately,
            # rather than leaving it blocked until its transaction timeout.
            cancelled_ids = {"first": [], "second": []}
            cancelled_results = {}
            waiter_started = {"first": threading.Event(),
                              "second": threading.Event()}

            def wait_until_cancelled(name):
                def emit_cancel(request_id):
                    cancelled_ids[name].append(request_id)
                    waiter_started[name].set()
                cancelled_results[name] = bus._await(
                    emit_cancel,
                    lambda ack, rid: ack == "ok cancel %s %d" % (name, rid),
                    10.0)

            cancelled_waiters = [
                threading.Thread(target=wait_until_cancelled, args=("first",)),
                threading.Thread(target=wait_until_cancelled, args=("second",)),
            ]
            for waiter in cancelled_waiters:
                waiter.start()
            for name in ("first", "second"):
                self.assertTrue(waiter_started[name].wait(timeout=2.0),
                                "%s cancellable waiter did not start" % name)
            cancel_deadline = time.monotonic() + 2.0
            while True:
                with bus._ack_lock:
                    registered = len(bus._ack_events)
                if registered == 2:
                    break
                self.assertLess(time.monotonic(), cancel_deadline,
                                "both waiter Events were not registered")
                time.sleep(0.005)
            bus.cancel_waiters()
            bus.cancel_waiters()  # cancellation is idempotent
            for waiter in cancelled_waiters:
                waiter.join(timeout=1.0)
                self.assertFalse(waiter.is_alive(),
                                 "cancel_waiters did not wake every waiter")
            self.assertEqual(cancelled_results, {"first": False, "second": False})
            with bus._ack_lock:
                for name in ("first", "second"):
                    request_id = cancelled_ids[name][0]
                    self.assertNotIn(request_id, bus.pending_requests)
                    self.assertNotIn(request_id, bus.ack_by_request)
                self.assertFalse(bus._ack_events)

            # cancel_waiters() is terminal for a CarryBus instance. Use a
            # separate live instance so this late-ACK test is independent of
            # the cancellation test above while sharing the same ROS executor.
            fresh_bus = CarryBus(node, executor_managed=True)
            fresh_waiter_ready = threading.Event()
            fresh_result = {}
            fresh_ids = []

            def fresh_emit(request_id):
                fresh_ids.append(request_id)
                fresh_waiter_ready.set()

            fresh_waiter = threading.Thread(target=lambda: fresh_result.setdefault(
                "ok", fresh_bus._await(
                    fresh_emit,
                    lambda ack, rid: ack == "ok fresh %d" % rid,
                    2.0)))
            fresh_waiter.start()
            self.assertTrue(fresh_waiter_ready.wait(timeout=2.0),
                            "fresh waiter did not register")
            self.assertEqual(len(fresh_ids), 1)
            fresh_id = fresh_ids[0]
            self.assertGreater(fresh_id, timed_out_id)

            ack_pub.publish(String(
                data="ok timeout %d" % timed_out_id))
            time.sleep(0.15)
            with fresh_bus._ack_lock:
                self.assertIn(fresh_id, fresh_bus.pending_requests)
                self.assertNotIn(timed_out_id, fresh_bus.ack_by_request)
                self.assertNotIn(fresh_id, fresh_bus.ack_by_request)

            ack_pub.publish(String(data="ok fresh %d" % fresh_id))
            fresh_waiter.join(timeout=3.0)
            self.assertFalse(fresh_waiter.is_alive(),
                             "fresh waiter did not finish after matching ACK")
            self.assertEqual(fresh_result, {"ok": True})
            with fresh_bus._ack_lock:
                self.assertNotIn(fresh_id, fresh_bus.pending_requests)
                self.assertNotIn(fresh_id, fresh_bus.ack_by_request)
                self.assertNotIn(fresh_id, fresh_bus._ack_events)
            fresh_bus.cancel_waiters()
        finally:
            # Executor lifecycle is part of the integration contract: stop the
            # sole spin owner before destroying its node or shutting ROS down.
            shutdown_ok = executor.shutdown(timeout_sec=2.0)
            spin_thread.join(timeout=2.0)
            self.assertTrue(shutdown_ok, "ROS 2 executor did not shut down cleanly")
            self.assertFalse(spin_thread.is_alive(),
                             "executor spin thread survived shutdown")
            executor.remove_node(node)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    unittest.main()
