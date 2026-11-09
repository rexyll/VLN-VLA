#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""CarryBus ↔ ros_gz_bridge ↔ Gazebo GraspCarry transaction test.

Run after starting the GraspCarry test world and a parameter_bridge instance
whose ACK output is routed to /drink/carry/ack_raw. The relay intentionally
drops the first ACK so CarryBus must retry and the plugin must replay its
completed transaction result.
"""
import ipaddress
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


def _bridge_gz_transport_peer(bridge_pid, sim_pid):
    """Find the bridge socket connected to a Gazebo-server listening port."""
    established = subprocess.check_output(
        ["sudo", "ss", "-Htnp", "state", "established"],
        text=True, stderr=subprocess.STDOUT)
    listeners = subprocess.check_output(
        ["sudo", "ss", "-Hltnp", "state", "listening"],
        text=True, stderr=subprocess.STDOUT)
    process_table = subprocess.check_output(
        ["ps", "-eo", "pid=,ppid=,args="], text=True)
    children = {}
    for process_line in process_table.splitlines():
        fields = process_line.strip().split(None, 2)
        if len(fields) >= 2:
            children.setdefault(int(fields[1]), set()).add(int(fields[0]))
    def descendants(root_pid):
        pids = {root_pid}
        frontier = [root_pid]
        while frontier:
            parent = frontier.pop()
            for child in children.get(parent, ()):
                if child not in pids:
                    pids.add(child)
                    frontier.append(child)
        return pids

    sim_pids = descendants(sim_pid)
    bridge_pids = descendants(bridge_pid)

    sim_ports = set()
    for line in listeners.splitlines():
        if not any("pid=%d," % pid in line for pid in sim_pids):
            continue
        fields = line.split()
        # ss -l output has: Recv-Q Send-Q Local-Address Peer-Address users.
        if len(fields) < 3:
            continue
        endpoint = fields[2]
        try:
            sim_ports.add(int(endpoint.rsplit(":", 1)[1]))
        except ValueError:
            continue

    peers = set()
    for line in established.splitlines():
        if not any("pid=%d," % pid in line for pid in bridge_pids):
            continue
        fields = line.split()
        # `ss -Htnp state established` omits the state column:
        # Recv-Q Send-Q Local-Address Peer-Address Process.
        # The peer endpoint is therefore fields[3]; fields[4] is the
        # optional process descriptor (for example, "users:(...)").
        if len(fields) < 4:
            continue
        endpoint = fields[3]
        host, port_text = endpoint.rsplit(":", 1)
        host = host.strip("[]")
        try:
            ipaddress.ip_address(host)
            port = int(port_text)
        except ValueError:
            continue
        # The ACK subscriber is an outgoing bridge connection to a Gazebo
        # publisher's listening port. The command path is the inverse: its
        # accepted bridge socket sees a Gazebo ephemeral client port.
        if port in sim_ports:
            peers.add((host, port))
    if len(peers) != 1:
        raise AssertionError(
            "expected one bridge-to-Gazebo publisher socket (bridge PID %d, "
            "Gazebo process tree %r); found %r; Gazebo listening ports: %r; "
            "established sockets: %s; listeners: %s" %
            (bridge_pid, sorted(sim_pids), sorted(peers), sorted(sim_ports),
             established, listeners))
    return next(iter(peers))


class AckRelay(Node):
    def __init__(self):
        super().__init__("grasp_carry_ack_fault_injector")
        self.publisher = self.create_publisher(String, ACK_TOPIC, 20)
        self.dropped_first = threading.Event()
        self.forwarded = []
        self.drop_next = threading.Event()
        self.dropped_next = threading.Event()
        self.drop_all = threading.Event()
        self.subscription = self.create_subscription(
            String, RAW_ACK_TOPIC, self._on_ack, 20)

    def _on_ack(self, message):
        if not self.dropped_first.is_set():
            self.dropped_first.set()
            self.get_logger().warning(
                "intentionally dropped first GraspCarry terminal ACK")
            return
        if self.drop_all.is_set():
            self.dropped_next.set()
            self.get_logger().warning(
                "intentionally dropped ACK while recovery gate is closed")
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
            relay.drop_all.set()
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
            os.killpg(bridge_pid, signal.SIGTERM)
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
                relay.drop_all.clear()
                restart_worker.join(timeout=8.0)
                self.assertFalse(
                    restart_worker.is_alive(),
                    "CarryBus waiter hung across bridge restart")
                self.assertTrue(
                    restart_result.get("ok"),
                    "same-request retry did not recover after bridge restart")

                # A separate transaction during a real bridge outage must
                # reach one terminal timeout, release all waiter state, and
                # never let a later request reuse its request ID.
                os.killpg(restarted_bridge.pid, signal.SIGTERM)
                try:
                    restarted_bridge.wait(timeout=3.0)
                except subprocess.TimeoutExpired:
                    os.killpg(restarted_bridge.pid, signal.SIGKILL)
                    restarted_bridge.wait(timeout=3.0)

                relay.drop_all.set()
                timeout_result = {}

                def disconnected_transaction():
                    def emit_disconnected(request_id):
                        timeout_result["request_id"] = request_id
                        bus._send(
                            "joint cup water_joint 0.009 %d" % request_id)

                    timeout_result["ok"] = bus._await(
                        emit_disconnected,
                        lambda ack, request_id: ack ==
                        "ok joint cup water_joint %d" % request_id,
                        timeout=0.8)

                timeout_worker = threading.Thread(
                    target=disconnected_transaction, daemon=True)
                timeout_worker.start()
                timeout_worker.join(timeout=3.0)
                self.assertFalse(
                    timeout_worker.is_alive(),
                    "CarryBus waiter did not terminate during bridge outage")
                self.assertFalse(
                    timeout_result.get("ok"),
                    "transaction unexpectedly succeeded with bridge stopped")
                timed_out_id = timeout_result.get("request_id", 0)
                self.assertGreater(timed_out_id, 0)
                self.assertFalse(bus.pending_requests)
                self.assertFalse(bus.ack_by_request)
                self.assertFalse(bus._ack_events)

                # Recreate the bridge and prove a fresh request receives a
                # distinct ID and a clean terminal ACK after the timeout.
                recovery_log = "/tmp/grasp-carry-bridge-after-timeout.log"
                with open(recovery_log, "w", encoding="utf-8") as stream:
                    recovered_bridge = subprocess.Popen(
                        ["ros2", "run", "ros_gz_bridge", "parameter_bridge",
                         "--ros-args", "-p",
                         "config_file:=" + bridge_config],
                        stdout=stream, stderr=subprocess.STDOUT,
                        start_new_session=True)
                try:
                    recovery_deadline = time.monotonic() + 15.0
                    while time.monotonic() < recovery_deadline:
                        if recovered_bridge.poll() is not None:
                            with open(recovery_log, encoding="utf-8") as log:
                                self.fail("recovered bridge exited: " + log.read())
                        with open(recovery_log, encoding="utf-8") as log:
                            recovery_output = log.read()
                        if ("Creating GZ->ROS Bridge" in recovery_output and
                                "Creating ROS->GZ Bridge" in recovery_output and
                                relay.count_publishers(RAW_ACK_TOPIC) >= 1 and
                                bus.pub.get_subscription_count() > 0):
                            break
                        time.sleep(0.1)
                    else:
                        with open(recovery_log, encoding="utf-8") as log:
                            self.fail("bridge after timeout not ready: " + log.read())

                    relay.drop_all.clear()
                    recovered_result = {}

                    def recovered_transaction():
                        def emit_recovered(request_id):
                            recovered_result["request_id"] = request_id
                            bus._send(
                                "place cup 0 0 0.5 0 0 0 1 %d" % request_id)

                        recovered_result["ok"] = bus._await(
                            emit_recovered,
                            lambda ack, request_id: ack ==
                            "ok place cup %d" % request_id,
                            timeout=6.0)

                    recovered_worker = threading.Thread(
                        target=recovered_transaction, daemon=True)
                    recovered_worker.start()
                    recovered_worker.join(timeout=8.0)
                    self.assertFalse(
                        recovered_worker.is_alive(),
                        "new CarryBus transaction hung after bridge recovery")
                    self.assertTrue(
                        recovered_result.get("ok"),
                        "new request did not succeed after bridge recovery")
                    self.assertGreater(
                        recovered_result.get("request_id", 0), timed_out_id,
                        "request ID was reused after timeout")
                finally:
                    try:
                        os.killpg(recovered_bridge.pid, signal.SIGTERM)
                        recovered_bridge.wait(timeout=3.0)
                    except ProcessLookupError:
                        pass
                    except subprocess.TimeoutExpired:
                        os.killpg(recovered_bridge.pid, signal.SIGKILL)
                        recovered_bridge.wait(timeout=3.0)

            finally:
                try:
                    os.killpg(restarted_bridge.pid, signal.SIGTERM)
                    restarted_bridge.wait(timeout=3.0)
                except ProcessLookupError:
                    pass
                except subprocess.TimeoutExpired:
                    os.killpg(restarted_bridge.pid, signal.SIGKILL)
                    restarted_bridge.wait(timeout=3.0)


            # Start a dedicated bridge for the transport-partition scenario.
            # Earlier bridge-restart/outage checks intentionally terminate their
            # processes, so their original PID must not be reused here.
            transport_bridge_log = "/tmp/grasp-carry-bridge-transport.log"
            with open(transport_bridge_log, "w", encoding="utf-8") as stream:
                transport_bridge = subprocess.Popen(
                    ["ros2", "run", "ros_gz_bridge", "parameter_bridge",
                     "--ros-args", "-p", "config_file:=" + bridge_config],
                    stdout=stream, stderr=subprocess.STDOUT,
                    start_new_session=True)
            try:
                transport_ready_deadline = time.monotonic() + 15.0
                while time.monotonic() < transport_ready_deadline:
                    if transport_bridge.poll() is not None:
                        with open(transport_bridge_log, encoding="utf-8") as log:
                            self.fail("transport bridge exited: " + log.read())
                    with open(transport_bridge_log, encoding="utf-8") as log:
                        transport_output = log.read()
                    if ("Creating GZ->ROS Bridge" in transport_output and
                            "Creating ROS->GZ Bridge" in transport_output and
                            relay.count_publishers(RAW_ACK_TOPIC) >= 1 and
                            bus.pub.get_subscription_count() > 0):
                        break
                    time.sleep(0.1)
                else:
                    with open(transport_bridge_log, encoding="utf-8") as log:
                        self.fail("transport bridge not ready: " + log.read())

                # Partition only the established Gazebo->bridge ACK data socket.
                # The bridge process and ROS side remain alive; a direct Gazebo
                # subscriber independently proves that the plugin still emits
                # terminal ACKs while this bridge's GZ transport socket is blocked.
                peer_ip, peer_port = _bridge_gz_transport_peer(
                    transport_bridge.pid, int(os.environ["CARRY_SIM_PID"]))
                iptables = "ip6tables" if ":" in peer_ip else "iptables"
                rule = ["-p", "tcp", "-d", peer_ip, "--dport",
                        str(peer_port), "-j", "DROP"]
                gz_env = os.environ.copy()
                saved_gz_config = gz_env.pop("CARRY_GZ_CONFIG_PATH", "")
                if saved_gz_config:
                    gz_env["GZ_CONFIG_PATH"] = saved_gz_config
                else:
                    gz_env.pop("GZ_CONFIG_PATH", None)
                observer_log = "/tmp/grasp-carry-gz-ack-observer.log"
                with open(observer_log, "w", encoding="utf-8") as observer_stream:
                    observer = subprocess.Popen(
                        ["gz", "topic", "-e", "-t", "/drink/carry/ack"],
                        stdout=observer_stream, stderr=subprocess.STDOUT,
                        env=gz_env)
                    firewall_installed = False
                    try:
                        time.sleep(1.0)
                        self.assertIsNone(observer.poll(),
                                          "Gazebo ACK observer exited early")
                        subprocess.check_call(
                            ["sudo", iptables, "-I", "OUTPUT", "1"] + rule)
                        firewall_installed = True
                        os.kill(transport_bridge.pid, 0)
                        transport_result = {}

                        def transport_disconnected_transaction():
                            def emit(request_id):
                                transport_result["request_id"] = request_id
                                bus._send(
                                    "joint cup water_joint 0.011 %d" % request_id)
                            transport_result["ok"] = bus._await(
                                emit,
                                lambda ack, request_id: ack ==
                                "ok joint cup water_joint %d" % request_id,
                                timeout=1.2)

                        transport_worker = threading.Thread(
                            target=transport_disconnected_transaction,
                            daemon=True)
                        transport_worker.start()
                        transport_worker.join(timeout=3.0)
                        self.assertFalse(
                            transport_worker.is_alive(),
                            "CarryBus waiter hung during Gazebo ACK transport partition")
                        self.assertFalse(
                            transport_result.get("ok"),
                            "ACK unexpectedly crossed the firewalled GZ transport socket")
                        transport_id = transport_result.get("request_id", 0)
                        self.assertGreater(transport_id, 0)
                        os.kill(transport_bridge.pid, 0)
                        self.assertFalse(bus.pending_requests)
                        self.assertFalse(bus.ack_by_request)
                        self.assertFalse(bus._ack_events)
                        observer_deadline = time.monotonic() + 2.0
                        while time.monotonic() < observer_deadline:
                            with open(observer_log, encoding="utf-8") as stream:
                                observed = stream.read()
                            if "ok joint cup water_joint %d" % transport_id in observed:
                                break
                            time.sleep(0.05)
                        else:
                            self.fail(
                                "Gazebo plugin did not publish ACK while bridge "
                                "transport socket was blocked; observer output: " +
                                observed)
                    finally:
                        if firewall_installed:
                            subprocess.check_call(
                                ["sudo", iptables, "-D", "OUTPUT"] + rule)
                        observer.terminate()
                        try:
                            observer.wait(timeout=2.0)
                        except subprocess.TimeoutExpired:
                            observer.kill()
                            observer.wait(timeout=2.0)

                # Removing the packet filter must restore the same bridge
                # process without restart; verify using a fresh request ID.
                restored_result = {}

                def restored_transport_transaction():
                    def emit(request_id):
                        restored_result["request_id"] = request_id
                        bus._send(
                            "place cup 0 0 0.5 0 0 0 1 %d" % request_id)
                    restored_result["ok"] = bus._await(
                        emit,
                        lambda ack, request_id: ack ==
                        "ok place cup %d" % request_id,
                        timeout=6.0)

                restored_worker = threading.Thread(
                    target=restored_transport_transaction, daemon=True)
                restored_worker.start()
                restored_worker.join(timeout=8.0)
                self.assertFalse(
                    restored_worker.is_alive(),
                    "CarryBus waiter hung after GZ transport recovery")
                self.assertTrue(
                    restored_result.get("ok"),
                    "GZ transport did not recover after firewall removal")
                self.assertGreater(restored_result["request_id"], transport_id)
                os.kill(transport_bridge.pid, 0)
                self.assertFalse(bus.pending_requests)
                self.assertFalse(bus.ack_by_request)
                self.assertFalse(bus._ack_events)
            finally:
                try:
                    os.killpg(transport_bridge.pid, signal.SIGTERM)
                    transport_bridge.wait(timeout=3.0)
                except ProcessLookupError:
                    pass
                except subprocess.TimeoutExpired:
                    os.killpg(transport_bridge.pid, signal.SIGKILL)
                    transport_bridge.wait(timeout=3.0)

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
