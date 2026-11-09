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
import re
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


def _bridge_gz_transport_peer(
        bridge_pid, sim_pid, topic, gz_config_path,
        endpoint_kind="publisher"):
    """Find the bridge socket connected to a Gazebo publisher or subscriber."""
    gz_env = os.environ.copy()
    gz_env.pop("CARRY_GZ_CONFIG_PATH", None)
    # Preserve the active ROS/Gazebo CLI configuration inherited from the
    # workflow. The pre-ROS environment may not define GZ_CONFIG_PATH at all;
    # deleting the post-source value makes gz-tools unable to locate commands.
    if gz_config_path:
        gz_env["GZ_CONFIG_PATH"] = gz_config_path
    topic_command = ["gz", "topic", "-i", "-t", topic]
    topic_info = ""
    topic_error = ""
    # Gazebo Transport discovery can briefly return CLI exit 255 while the
    # server is alive and publishers are being re-announced after bridge
    # restarts. Retry the read-only metadata query, but retain the final
    # command output in the assertion if discovery never becomes available.
    for attempt in range(10):
        try:
            topic_info = subprocess.check_output(
                topic_command, text=True, stderr=subprocess.STDOUT,
                env=gz_env, timeout=5.0)
            break
        except (subprocess.CalledProcessError, subprocess.TimeoutExpired) as exc:
            output = getattr(exc, "output", "") or ""
            if isinstance(output, bytes):
                output = output.decode("utf-8", errors="replace")
            topic_error = str(exc) + (": " + output if output else "")
            if attempt == 9:
                raise AssertionError(
                    "Gazebo topic metadata query failed for %s after retries; "
                    "GZ_CONFIG_PATH=%r; last error: %s" %
                    (topic, gz_env.get("GZ_CONFIG_PATH"), topic_error))
            time.sleep(0.25)
    topic_sections = topic_info.split("Subscribers", 1)
    if endpoint_kind == "subscriber":
        endpoint_section = topic_sections[1] if len(topic_sections) > 1 else ""
    elif endpoint_kind == "publisher":
        endpoint_section = topic_sections[0]
    else:
        raise ValueError("endpoint_kind must be publisher or subscriber")
    publisher_endpoints = set()
    for address, port_text in re.findall(r"tcp://(\[[^]]+\]|[^:,\s]+):(\d+)", endpoint_section):
        host = address.strip("[]")
        try:
            ipaddress.ip_address(host)
            publisher_endpoints.add((host, int(port_text)))
        except ValueError:
            continue
    established = subprocess.check_output(["sudo", "ss", "-Htnp", "state", "established"], text=True, stderr=subprocess.STDOUT)
    listeners = subprocess.check_output(["sudo", "ss", "-Hltnp", "state", "listening"], text=True, stderr=subprocess.STDOUT)
    process_table = subprocess.check_output(["ps", "-eo", "pid=,ppid=,args="], text=True)
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
    peers = set()
    for line in established.splitlines():
        if not any("pid=%d," % pid in line for pid in bridge_pids):
            continue
        fields = line.split()
        if len(fields) < 4:
            continue
        local_endpoint, endpoint = fields[2], fields[3]
        local_host, local_port_text = local_endpoint.rsplit(":", 1)
        host, port_text = endpoint.rsplit(":", 1)
        local_host = local_host.strip("[]")
        host = host.strip("[]")
        try:
            ipaddress.ip_address(local_host)
            local_port = int(local_port_text)
            ipaddress.ip_address(host)
            port = int(port_text)
        except ValueError:
            continue
        if (host, port) in publisher_endpoints:
            peers.add((local_host, local_port, host, port))
    if len(peers) != 1:
        raise AssertionError(
            "expected one bridge socket for Gazebo topic %s (bridge PID %d, "
            "Gazebo process tree %r); found %r; topic info: %s; "
            "established sockets: %s" %
            (topic, bridge_pid, sorted(sim_pids), sorted(peers), topic_info,
             established))
    return next(iter(peers))



def _gazebo_topic_endpoints(topic, gz_config_path, endpoint_kind):
    """Return the advertised TCP endpoints for one Gazebo topic direction."""
    env = os.environ.copy()
    env.pop("CARRY_GZ_CONFIG_PATH", None)
    if gz_config_path:
        env["GZ_CONFIG_PATH"] = gz_config_path
    try:
        info = subprocess.check_output(
            ["gz", "topic", "-i", "-t", topic],
            text=True, stderr=subprocess.STDOUT, env=env, timeout=5.0)
    except (subprocess.CalledProcessError, subprocess.TimeoutExpired):
        return set()
    sections = info.split("Subscribers", 1)
    if endpoint_kind == "publisher":
        section = sections[0]
    elif endpoint_kind == "subscriber":
        section = sections[1] if len(sections) > 1 else ""
    else:
        raise ValueError("endpoint_kind must be publisher or subscriber")
    endpoints = set()
    for address, port_text in re.findall(
            r"tcp://(\[[^]]+\]|[^:,\s]+):(\d+)", section):
        host = address.strip("[]")
        try:
            ipaddress.ip_address(host)
            endpoints.add((host, int(port_text)))
        except ValueError:
            continue
    return endpoints


def _signal_process_group(pid, signum):
    """Signal a launcher process's actual group, tolerating setsid wrappers."""
    try:
        pgid = os.getpgid(pid)
    except ProcessLookupError:
        return False
    try:
        os.killpg(pgid, signum)
    except ProcessLookupError:
        # Some launchers may have detached their child into a new session.
        try:
            os.kill(pid, signum)
        except ProcessLookupError:
            return False
    return True


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
            _signal_process_group(bridge_pid, signal.SIGTERM)
            deadline = time.monotonic() + 5.0
            while time.monotonic() < deadline:
                try:
                    os.kill(bridge_pid, 0)
                except ProcessLookupError:
                    break
                time.sleep(0.05)
            else:
                _signal_process_group(bridge_pid, signal.SIGKILL)
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

                # Reload only the GraspCarry system plugin by removing and
                # recreating its dedicated model entity. Gazebo server and
                # parameter_bridge must remain the same live processes.
                sim_pid = int(os.environ["CARRY_SIM_PID"])
                bridge_pid = transport_bridge.pid
                ack_peer_before = _bridge_gz_transport_peer(
                    bridge_pid, sim_pid, "/drink/carry/ack",
                    os.environ.get("CARRY_GZ_CONFIG_PATH", ""))
                command_peer_before = _bridge_gz_transport_peer(
                    bridge_pid, sim_pid, "/drink/carry",
                    os.environ.get("CARRY_GZ_CONFIG_PATH", ""),
                    endpoint_kind="subscriber")
                plugin_gz_env = os.environ.copy()
                plugin_saved_gz_config = plugin_gz_env.pop(
                    "CARRY_GZ_CONFIG_PATH", "")
                if plugin_saved_gz_config:
                    plugin_gz_env["GZ_CONFIG_PATH"] = plugin_saved_gz_config
                else:
                    plugin_gz_env.pop("GZ_CONFIG_PATH", None)
                relay.drop_all.set()
                relay.dropped_next.clear()
                plugin_reload_result = {}
                plugin_reload_send = {"last": 0.0}

                def plugin_reload_transaction():
                    def emit(request_id):
                        now = time.monotonic()
                        if now - plugin_reload_send["last"] < 0.5:
                            return
                        plugin_reload_send["last"] = now
                        plugin_reload_result["request_id"] = request_id
                        bus._send(
                            "place cup 0.11 0.03 0.52 0 0 0 1 %d" % request_id)
                    plugin_reload_result["ok"] = bus._await(
                        emit,
                        lambda ack, request_id: ack ==
                        "ok place cup %d" % request_id,
                        timeout=30.0)

                plugin_reload_worker = threading.Thread(
                    target=plugin_reload_transaction, daemon=True)
                plugin_reload_worker.start()
                self.assertTrue(
                    relay.dropped_next.wait(8.0),
                    "plugin terminal ACK was not intercepted before reload")

                remove_output = subprocess.check_output(
                    ["gz", "service", "-s",
                     "/world/grasp_carry_test/remove",
                     "--reqtype", "gz.msgs.Entity",
                     "--reptype", "gz.msgs.Boolean", "--timeout", "2000",
                     "--req",
                     'name: "grasp_carry_plugin_host", type: MODEL'],
                    text=True, stderr=subprocess.STDOUT, env=plugin_gz_env,
                    timeout=5.0)
                self.assertIn("data: true", remove_output)
                endpoint_deadline = time.monotonic() + 8.0
                while time.monotonic() < endpoint_deadline:
                    ack_endpoints = _gazebo_topic_endpoints(
                        "/drink/carry/ack",
                        os.environ.get("CARRY_GZ_CONFIG_PATH", ""),
                        "publisher")
                    command_endpoints = _gazebo_topic_endpoints(
                        "/drink/carry",
                        os.environ.get("CARRY_GZ_CONFIG_PATH", ""),
                        "subscriber")
                    if not ack_endpoints and not command_endpoints:
                        break
                    time.sleep(0.1)
                else:
                    self.fail(
                        "GraspCarry endpoints remained advertised after "
                        "removing its model/plugin")
                os.kill(sim_pid, 0)
                os.kill(bridge_pid, 0)

                host_sdf = os.path.join(
                    ROOT, "tests", "grasp_carry_plugin_host.sdf")
                create_output = subprocess.check_output(
                    ["gz", "service", "-s",
                     "/world/grasp_carry_test/create",
                     "--reqtype", "gz.msgs.EntityFactory",
                     "--reptype", "gz.msgs.Boolean", "--timeout", "5000",
                     "--req",
                     'sdf_filename: "%s", name: "grasp_carry_plugin_host"'
                     % host_sdf],
                    text=True, stderr=subprocess.STDOUT, env=plugin_gz_env,
                    timeout=8.0)
                self.assertIn("data: true", create_output)
                endpoint_deadline = time.monotonic() + 15.0
                while time.monotonic() < endpoint_deadline:
                    ack_endpoints = _gazebo_topic_endpoints(
                        "/drink/carry/ack",
                        os.environ.get("CARRY_GZ_CONFIG_PATH", ""),
                        "publisher")
                    command_endpoints = _gazebo_topic_endpoints(
                        "/drink/carry",
                        os.environ.get("CARRY_GZ_CONFIG_PATH", ""),
                        "subscriber")
                    if ack_endpoints and command_endpoints:
                        break
                    time.sleep(0.1)
                else:
                    self.fail(
                        "GraspCarry endpoints did not return after "
                        "recreating its model/plugin")
                ack_peer_after = _bridge_gz_transport_peer(
                    bridge_pid, sim_pid, "/drink/carry/ack",
                    os.environ.get("CARRY_GZ_CONFIG_PATH", ""))
                command_peer_after = _bridge_gz_transport_peer(
                    bridge_pid, sim_pid, "/drink/carry",
                    os.environ.get("CARRY_GZ_CONFIG_PATH", ""),
                    endpoint_kind="subscriber")
                os.kill(sim_pid, 0)
                os.kill(bridge_pid, 0)
                relay.drop_all.clear()
                plugin_reload_worker.join(timeout=32.0)
                self.assertFalse(
                    plugin_reload_worker.is_alive(),
                    "CarryBus waiter hung across plugin-only reload")
                self.assertTrue(
                    plugin_reload_result.get("ok"),
                    "same-ID transaction did not recover after plugin-only reload")
                self.assertGreater(plugin_reload_result.get("request_id", 0), 0)
                self.assertFalse(bus.pending_requests)
                self.assertFalse(bus.ack_by_request)
                self.assertFalse(bus._ack_events)

                fresh_plugin_result = {}

                def fresh_after_plugin_reload():
                    def emit(request_id):
                        fresh_plugin_result["request_id"] = request_id
                        bus._send(
                            "place cup 0.10 0.03 0.52 0 0 0 1 %d" % request_id)
                    fresh_plugin_result["ok"] = bus._await(
                        emit,
                        lambda ack, request_id: ack ==
                        "ok place cup %d" % request_id,
                        timeout=8.0)

                fresh_plugin_worker = threading.Thread(
                    target=fresh_after_plugin_reload, daemon=True)
                fresh_plugin_worker.start()
                fresh_plugin_worker.join(timeout=10.0)
                self.assertFalse(
                    fresh_plugin_worker.is_alive(),
                    "fresh transaction hung after plugin-only reload")
                self.assertTrue(
                    fresh_plugin_result.get("ok"),
                    "fresh transaction failed after plugin-only reload")
                self.assertGreater(
                    fresh_plugin_result.get("request_id", 0),
                    plugin_reload_result["request_id"])
                self.assertFalse(bus.pending_requests)
                self.assertFalse(bus.ack_by_request)
                self.assertFalse(bus._ack_events)
                os.kill(sim_pid, 0)
                os.kill(bridge_pid, 0)

                # Partition only the established Gazebo->bridge ACK data socket.
                # The bridge process and ROS side remain alive; a direct Gazebo
                # subscriber independently proves that the plugin still emits
                # terminal ACKs while this bridge's GZ transport socket is blocked.
                local_ip, local_port, peer_ip, peer_port = _bridge_gz_transport_peer(
                    transport_bridge.pid, int(os.environ["CARRY_SIM_PID"]),
                    "/drink/carry/ack",
                    os.environ.get("CARRY_GZ_CONFIG_PATH", ""))
                iptables = "ip6tables" if ":" in peer_ip else "iptables"
                # ACK payloads travel from the Gazebo publisher endpoint into
                # the bridge's local TCP socket. Match that exact destination
                # port in INPUT; OUTPUT to the remote publisher endpoint does
                # not block the return data path.
                rule = ["-p", "tcp", "-s", peer_ip, "--sport",
                        str(peer_port), "-d", local_ip, "--dport",
                        str(local_port), "-j", "DROP"]
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
                            ["sudo", iptables, "-I", "INPUT", "1"] + rule)
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
                                ["sudo", iptables, "-D", "INPUT"] + rule)
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

                # Block both directions on both live Gazebo Transport TCP
                # peers (ROS->Gazebo command subscriber and Gazebo->ROS ACK
                # publisher) while keeping the bridge and simulator processes
                # alive. Independent native-GZ observers prove that the
                # request never reaches the plugin and no ACK crosses back.
                command_peer = _bridge_gz_transport_peer(
                    transport_bridge.pid, int(os.environ["CARRY_SIM_PID"]),
                    "/drink/carry",
                    os.environ.get("CARRY_GZ_CONFIG_PATH", ""),
                    endpoint_kind="subscriber")
                ack_peer = _bridge_gz_transport_peer(
                    transport_bridge.pid, int(os.environ["CARRY_SIM_PID"]),
                    "/drink/carry/ack",
                    os.environ.get("CARRY_GZ_CONFIG_PATH", ""))
                bidirectional_observer_log = (
                    "/tmp/grasp-carry-gz-bidirectional-observer.log")
                bidirectional_ack_log = (
                    "/tmp/grasp-carry-gz-bidirectional-ack.log")
                observer_env = os.environ.copy()
                observer_config = observer_env.pop(
                    "CARRY_GZ_CONFIG_PATH", "")
                if observer_config:
                    observer_env["GZ_CONFIG_PATH"] = observer_config
                else:
                    observer_env.pop("GZ_CONFIG_PATH", None)
                with open(bidirectional_observer_log, "w",
                          encoding="utf-8") as command_stream, open(
                              bidirectional_ack_log, "w",
                              encoding="utf-8") as ack_stream:
                    command_observer = subprocess.Popen(
                        ["gz", "topic", "-e", "-t", "/drink/carry"],
                        stdout=command_stream, stderr=subprocess.STDOUT,
                        env=observer_env)
                    ack_observer = subprocess.Popen(
                        ["gz", "topic", "-e", "-t", "/drink/carry/ack"],
                        stdout=ack_stream, stderr=subprocess.STDOUT,
                        env=observer_env)
                    firewall_rules = []
                    try:
                        time.sleep(1.0)
                        self.assertIsNone(
                            command_observer.poll(),
                            "Gazebo command observer exited early")
                        self.assertIsNone(
                            ack_observer.poll(),
                            "Gazebo ACK observer exited early")
                        for local_host, local_port, remote_host, remote_port in (
                                command_peer, ack_peer):
                            family = (
                                "ip6tables" if ":" in remote_host
                                else "iptables")
                            inbound = [
                                "-p", "tcp", "-s", remote_host,
                                "--sport", str(remote_port),
                                "-d", local_host, "--dport",
                                str(local_port), "-j", "DROP"]
                            outbound = [
                                "-p", "tcp", "-s", local_host,
                                "--sport", str(local_port),
                                "-d", remote_host, "--dport",
                                str(remote_port), "-j", "DROP"]
                            for chain, rule_args in (
                                    ("INPUT", inbound),
                                    ("OUTPUT", outbound)):
                                subprocess.check_call(
                                    ["sudo", family, "-I", chain, "1"] +
                                    rule_args)
                                firewall_rules.append(
                                    (family, chain, rule_args))
                        os.kill(transport_bridge.pid, 0)
                        os.kill(int(os.environ["CARRY_SIM_PID"]), 0)
                        bidirectional_result = {}

                        def bidirectional_partition_transaction():
                            def emit(request_id):
                                bidirectional_result["request_id"] = request_id
                                bus._send(
                                    "joint cup water_joint 0.009 %d" %
                                    request_id)
                            bidirectional_result["ok"] = bus._await(
                                emit,
                                lambda ack, request_id: ack ==
                                "ok joint cup water_joint %d" % request_id,
                                timeout=12.0)

                        bidirectional_worker = threading.Thread(
                            target=bidirectional_partition_transaction,
                            daemon=True)
                        bidirectional_worker.start()
                        request_deadline = time.monotonic() + 3.0
                        while (time.monotonic() < request_deadline and
                               not bidirectional_result.get("request_id")):
                            time.sleep(0.01)
                        self.assertTrue(
                            bidirectional_result.get("request_id"),
                            "bidirectional partition request was not issued")
                        bidirectional_id = bidirectional_result["request_id"]
                        time.sleep(1.5)
                        with open(bidirectional_observer_log,
                                  encoding="utf-8") as stream:
                            observed_commands = stream.read()
                        with open(bidirectional_ack_log,
                                  encoding="utf-8") as stream:
                            observed_acks = stream.read()
                        self.assertNotIn(
                            "joint cup water_joint 0.009 %d" %
                            bidirectional_id, observed_commands,
                            "command crossed the blocked bridge->Gazebo peer")
                        self.assertNotIn(
                            "joint cup water_joint %d" % bidirectional_id,
                            observed_acks,
                            "plugin ACK appeared while both Transport peers "
                            "were firewalled")
                        os.kill(transport_bridge.pid, 0)
                        os.kill(int(os.environ["CARRY_SIM_PID"]), 0)
                    finally:
                        for family, chain, rule_args in reversed(
                                firewall_rules):
                            subprocess.check_call(
                                ["sudo", family, "-D", chain] + rule_args)
                        command_observer.terminate()
                        ack_observer.terminate()
                        for process in (command_observer, ack_observer):
                            try:
                                process.wait(timeout=2.0)
                            except subprocess.TimeoutExpired:
                                process.kill()
                                process.wait(timeout=2.0)
                bidirectional_worker.join(timeout=13.0)
                self.assertFalse(
                    bidirectional_worker.is_alive(),
                    "CarryBus waiter hung after bidirectional Transport recovery")
                self.assertTrue(
                    bidirectional_result.get("ok"),
                    "same request ID did not recover after bidirectional "
                    "Transport firewall removal")
                self.assertGreater(bidirectional_id, restored_result["request_id"])
                os.kill(transport_bridge.pid, 0)
                os.kill(int(os.environ["CARRY_SIM_PID"]), 0)
                self.assertFalse(bus.pending_requests)
                self.assertFalse(bus.ack_by_request)
                self.assertFalse(bus._ack_events)

                # Saturate the real GraspCarry queue through CarryBus ->
                # ROS 2 -> ros_gz_bridge -> Gazebo. The existing native-GZ
                # publisher test covers the plugin queue in isolation; this
                # scenario proves that queue-full terminal ACKs traverse the
                # production bridge path and that accepted work drains FIFO
                # after physics resumes.
                pause_response = subprocess.check_output(
                    ["gz", "service", "-s", "/world/grasp_carry_test/control",
                     "--reqtype", "gz.msgs.WorldControl",
                     "--reptype", "gz.msgs.Boolean", "--timeout", "1000",
                     "--req", "pause: true"],
                    text=True, stderr=subprocess.STDOUT, env=gz_env,
                    timeout=5.0)
                self.assertIn("data: true", pause_response)
                simulation_paused = True
                try:
                    # Let an in-progress PreUpdate finish before submitting
                    # more unique transactions than the plugin's 128 slots.
                    time.sleep(0.5)
                    queue_worker_count = 144
                    queue_start = threading.Barrier(queue_worker_count + 1)
                    queue_results = {}
                    queue_ids = {}
                    queue_workers = []

                    def submit_backpressure_transaction(index):
                        queue_start.wait(timeout=5.0)
                        # Spread publication over a bounded interval so the
                        # reliable ROS/DDS and bridge queues can deliver each
                        # distinct request instead of testing publisher burst
                        # loss rather than GraspCarry's bounded queue.
                        time.sleep(index * 0.003)

                        def emit(request_id):
                            queue_ids[index] = request_id
                            bus._send(
                                "joint cup water_joint 0.001 %d" % request_id)

                        queue_results[index] = bus._await(
                            emit,
                            lambda ack, request_id: ack ==
                            "ok joint cup water_joint %d" % request_id,
                            timeout=45.0)

                    for index in range(queue_worker_count):
                        worker = threading.Thread(
                            target=submit_backpressure_transaction,
                            args=(index,), daemon=True)
                        queue_workers.append(worker)
                        worker.start()
                    queue_start.wait(timeout=5.0)

                    queue_full_deadline = time.monotonic() + 7.0
                    while time.monotonic() < queue_full_deadline:
                        if any(" queue-full" in ack
                               for ack in relay.forwarded):
                            break
                        time.sleep(0.05)
                    else:
                        self.fail(
                            "real ROS/Gazebo bridge path did not return a "
                            "queue-full terminal ACK while simulation was paused")

                    unpause_response = subprocess.check_output(
                        ["gz", "service", "-s",
                         "/world/grasp_carry_test/control",
                         "--reqtype", "gz.msgs.WorldControl",
                         "--reptype", "gz.msgs.Boolean", "--timeout", "1000",
                         "--req", "pause: false"],
                        text=True, stderr=subprocess.STDOUT, env=gz_env,
                        timeout=5.0)
                    self.assertIn("data: true", unpause_response)
                    simulation_paused = False

                    for worker in queue_workers:
                        worker.join(timeout=50.0)
                        self.assertFalse(
                            worker.is_alive(),
                            "bridge backpressure transaction waiter hung")
                    self.assertEqual(len(queue_results), queue_worker_count)
                    self.assertEqual(len(queue_ids), queue_worker_count)

                    # Every request must have exactly one terminal payload
                    # identity (retries may replay the same payload). No
                    # request may silently disappear or change outcome.
                    observed_by_id = {}
                    for ack in relay.forwarded:
                        request_id = next(
                            (int(token) for token in reversed(ack.split())
                             if token.isdigit()), None)
                        if request_id in set(queue_ids.values()):
                            observed_by_id.setdefault(
                                request_id, set()).add(ack)
                    rejected = 0
                    for index, request_id in queue_ids.items():
                        payloads = observed_by_id.get(request_id, set())
                        self.assertEqual(
                            len(payloads), 1,
                            "request %d had missing/conflicting terminal ACKs: %r"
                            % (request_id, payloads))
                        payload = next(iter(payloads))
                        if payload.startswith("ok joint cup water_joint "):
                            self.assertTrue(queue_results[index])
                        else:
                            self.assertEqual(
                                payload,
                                "fail joint cup water_joint %d queue-full"
                                % request_id)
                            self.assertFalse(queue_results[index])
                            rejected += 1
                    self.assertGreater(
                        rejected, 0,
                        "bridge-path flood never exercised bounded queue rejection")
                    self.assertFalse(bus.pending_requests)
                    self.assertFalse(bus.ack_by_request)
                    self.assertFalse(bus._ack_events)

                    # The queue must be usable for a fresh transaction after
                    # the backlog has drained.
                    drained_result = {}
                    def drained_transaction():
                        def emit(request_id):
                            drained_result["request_id"] = request_id
                            bus._send(
                                "joint cup water_joint 0.002 %d" % request_id)
                        drained_result["ok"] = bus._await(
                            emit,
                            lambda ack, request_id: ack ==
                            "ok joint cup water_joint %d" % request_id,
                            timeout=5.0)
                    drained_worker = threading.Thread(
                        target=drained_transaction, daemon=True)
                    drained_worker.start()
                    drained_worker.join(timeout=7.0)
                    self.assertFalse(
                        drained_worker.is_alive(),
                        "fresh transaction hung after bridge queue drained")
                    self.assertTrue(
                        drained_result.get("ok"),
                        "bridge queue did not recover after backpressure")
                finally:
                    if simulation_paused:
                        subprocess.check_call(
                            ["gz", "service", "-s",
                             "/world/grasp_carry_test/control",
                             "--reqtype", "gz.msgs.WorldControl",
                             "--reptype", "gz.msgs.Boolean", "--timeout", "1000",
                             "--req", "pause: false"],
                            env=gz_env, timeout=5.0,
                            stdout=subprocess.DEVNULL,
                            stderr=subprocess.STDOUT)
            finally:
                try:
                    os.killpg(transport_bridge.pid, signal.SIGTERM)
                    transport_bridge.wait(timeout=3.0)
                except ProcessLookupError:
                    pass
                except subprocess.TimeoutExpired:
                    os.killpg(transport_bridge.pid, signal.SIGKILL)
                    transport_bridge.wait(timeout=3.0)


            # Whole Gazebo server restart (not plugin-only reload). The plugin
            # process and world are destroyed; bridge is restarted separately
            # because its old Transport endpoints are not assumed to reconnect.
            # The first ACK is observed but deliberately withheld from CarryBus,
            # leaving the transaction outcome unknown to the caller.
            relay.drop_all.set()
            relay.dropped_next.clear()
            pre_restart_bridge_log = (
                "/tmp/grasp-carry-bridge-before-server-restart.log")
            with open(pre_restart_bridge_log, "w", encoding="utf-8") as stream:
                pre_restart_bridge = subprocess.Popen(
                    ["ros2", "run", "ros_gz_bridge", "parameter_bridge",
                     "--ros-args", "-p", "config_file:=" + bridge_config],
                    stdout=stream, stderr=subprocess.STDOUT,
                    start_new_session=True)
            pre_bridge_deadline = time.monotonic() + 15.0
            while time.monotonic() < pre_bridge_deadline:
                if pre_restart_bridge.poll() is not None:
                    with open(pre_restart_bridge_log, encoding="utf-8") as log:
                        self.fail("pre-restart bridge exited: " + log.read())
                with open(pre_restart_bridge_log, encoding="utf-8") as log:
                    pre_bridge_output = log.read()
                if ("Creating GZ->ROS Bridge" in pre_bridge_output and
                        "Creating ROS->GZ Bridge" in pre_bridge_output and
                        relay.count_publishers(RAW_ACK_TOPIC) >= 1 and
                        bus.pub.get_subscription_count() > 0):
                    break
                time.sleep(0.1)
            else:
                with open(pre_restart_bridge_log, encoding="utf-8") as log:
                    self.fail("pre-restart bridge not ready: " + log.read())
            _bridge_gz_transport_peer(
                pre_restart_bridge.pid, int(os.environ["CARRY_SIM_PID"]),
                "/drink/carry/ack",
                os.environ.get("CARRY_GZ_CONFIG_PATH", ""))
            _bridge_gz_transport_peer(
                pre_restart_bridge.pid, int(os.environ["CARRY_SIM_PID"]),
                "/drink/carry", os.environ.get("CARRY_GZ_CONFIG_PATH", ""),
                endpoint_kind="subscriber")
            # Allow the independent ROS->GZ command socket to finish discovery
            # as well; bridge log creation alone is not a transport handshake.
            time.sleep(1.0)
            server_restart_result = {}
            server_restart_send = {"last": 0.0}

            def server_restart_transaction():
                def emit(request_id):
                    # _await polls at 80ms. During a server outage that rate
                    # can fill the plugin's bounded queue as soon as it returns.
                    # Keep retry semantics and request identity unchanged, but
                    # cap this recovery probe to 2Hz so one transaction cannot
                    # flood a freshly restarted plugin before ACK discovery.
                    now = time.monotonic()
                    if now - server_restart_send["last"] < 0.5:
                        return
                    server_restart_send["last"] = now
                    server_restart_result["request_id"] = request_id
                    bus._send(
                        "place cup 0.12 0.03 0.52 0 0 0 1 %d" % request_id)
                server_restart_result["ok"] = bus._await(
                    emit,
                    lambda ack, request_id: ack ==
                    "ok place cup %d" % request_id,
                    timeout=50.0)

            server_restart_worker = threading.Thread(
                target=server_restart_transaction, daemon=True)
            server_restart_worker.start()
            self.assertTrue(
                relay.dropped_next.wait(15.0),
                "pre-restart plugin terminal ACK was not intercepted")

            sim_pid = int(os.environ["CARRY_SIM_PID"])
            try:
                os.kill(sim_pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            deadline = time.monotonic() + 8.0
            while time.monotonic() < deadline:
                try:
                    os.kill(sim_pid, 0)
                except ProcessLookupError:
                    break
                time.sleep(0.05)
            else:
                os.kill(sim_pid, signal.SIGKILL)
                self.fail("Gazebo server did not stop for restart scenario")

            try:
                os.killpg(pre_restart_bridge.pid, signal.SIGTERM)
                pre_restart_bridge.wait(timeout=3.0)
            except ProcessLookupError:
                pass
            except subprocess.TimeoutExpired:
                os.killpg(pre_restart_bridge.pid, signal.SIGKILL)
                pre_restart_bridge.wait(timeout=3.0)

            restart_env = os.environ.copy()
            restart_env.pop("CARRY_GZ_CONFIG_PATH", None)
            saved_gz_config = os.environ.get("CARRY_GZ_CONFIG_PATH", "")
            # ROS setup leaves only vendor command manifests in the
            # active path, hiding the Gazebo Harmonic "sim" command. Restore
            # the actual gz-sim manifest directory first, then retain ROS
            # vendor manifests so both CLI command families remain available.
            sim_config_dir = os.environ.get(
                "CARRY_GZ_SIM_CONFIG_DIR", "")
            if not sim_config_dir or not os.path.isdir(sim_config_dir):
                self.fail(
                    "Gazebo sim CLI config directory is unavailable: %r" %
                    sim_config_dir)
            config_dirs = [sim_config_dir]
            if saved_gz_config:
                config_dirs.extend(
                    path for path in saved_gz_config.split(os.pathsep)
                    if path and path != sim_config_dir)
            restart_env["GZ_CONFIG_PATH"] = os.pathsep.join(config_dirs)
            probe_env = restart_env.copy()
            server_log = "/tmp/grasp-carry-gz-server-restarted.log"
            with open(server_log, "w", encoding="utf-8") as stream:
                restarted_sim = subprocess.Popen(
                    ["gz", "sim", "-s", "-r", "-v", "2",
                     "tests/grasp_carry_integration.sdf"],
                    stdout=stream, stderr=subprocess.STDOUT,
                    env=restart_env, start_new_session=True)
            restarted_server_bridge = None
            try:
                server_ready_deadline = time.monotonic() + 30.0
                while time.monotonic() < server_ready_deadline:
                    if restarted_sim.poll() is not None:
                        with open(server_log, encoding="utf-8") as log:
                            self.fail(
                        "restarted Gazebo exited (GZ_CONFIG_PATH=%r): %s" %
                        (restart_env.get("GZ_CONFIG_PATH"), log.read()))
                    try:
                        topics = subprocess.check_output(
                            ["gz", "topic", "-l"], text=True,
                            stderr=subprocess.STDOUT, env=probe_env,
                            timeout=4.0)
                    except (subprocess.CalledProcessError,
                            subprocess.TimeoutExpired):
                        time.sleep(0.25)
                        continue
                    if "/drink/carry/ack" in topics:
                        break
                    time.sleep(0.25)
                else:
                    with open(server_log, encoding="utf-8") as log:
                        self.fail("restarted Gazebo plugin not ready: " + log.read())

                restarted_bridge_log = (
                    "/tmp/grasp-carry-bridge-after-server-restart.log")
                with open(restarted_bridge_log, "w", encoding="utf-8") as stream:
                    restarted_server_bridge = subprocess.Popen(
                        ["ros2", "run", "ros_gz_bridge", "parameter_bridge",
                         "--ros-args", "-p",
                         "config_file:=" + bridge_config],
                        stdout=stream, stderr=subprocess.STDOUT,
                        start_new_session=True)
                bridge_ready_deadline = time.monotonic() + 20.0
                while time.monotonic() < bridge_ready_deadline:
                    if restarted_server_bridge.poll() is not None:
                        with open(restarted_bridge_log, encoding="utf-8") as log:
                            self.fail("bridge after Gazebo restart exited: " + log.read())
                    with open(restarted_bridge_log, encoding="utf-8") as log:
                        bridge_output = log.read()
                    if ("Creating GZ->ROS Bridge" in bridge_output and
                            "Creating ROS->GZ Bridge" in bridge_output and
                            relay.count_publishers(RAW_ACK_TOPIC) >= 1 and
                            bus.pub.get_subscription_count() > 0):
                        break
                    time.sleep(0.1)
                else:
                    with open(restarted_bridge_log, encoding="utf-8") as log:
                        self.fail("bridge after Gazebo restart not ready: " + log.read())

                # Require actual Gazebo Transport TCP peers in both directions;
                # bridge log creation and ROS graph matches alone are insufficient.
                _bridge_gz_transport_peer(
                    restarted_server_bridge.pid, restarted_sim.pid,
                    "/drink/carry/ack", saved_gz_config)
                _bridge_gz_transport_peer(
                    restarted_server_bridge.pid, restarted_sim.pid,
                    "/drink/carry", saved_gz_config,
                    endpoint_kind="subscriber")

                # The retry carries the original request ID. Since the entire
                # plugin instance was recreated, its in-memory ACK cache is
                # gone; this validates state-setting replay, not exactly-once.
                relay.drop_all.clear()
                server_restart_worker.join(timeout=52.0)
                self.assertFalse(
                    server_restart_worker.is_alive(),
                    "CarryBus waiter hung across whole Gazebo server restart")
                self.assertTrue(
                    server_restart_result.get("ok"),
                    "same-ID state-setting retry did not recover after "
                    "whole Gazebo server restart")
                self.assertGreater(server_restart_result.get("request_id", 0), 0)
                self.assertFalse(bus.pending_requests)
                self.assertFalse(bus.ack_by_request)
                self.assertFalse(bus._ack_events)

                fresh_after_server_restart = {}

                def fresh_after_restart_transaction():
                    def emit(request_id):
                        fresh_after_server_restart["request_id"] = request_id
                        bus._send(
                            "place cup 0.10 0.03 0.52 0 0 0 1 %d" % request_id)
                    fresh_after_server_restart["ok"] = bus._await(
                        emit,
                        lambda ack, request_id: ack ==
                        "ok place cup %d" % request_id,
                        timeout=8.0)

                fresh_worker = threading.Thread(
                    target=fresh_after_restart_transaction, daemon=True)
                fresh_worker.start()
                fresh_worker.join(timeout=10.0)
                self.assertFalse(
                    fresh_worker.is_alive(),
                    "fresh CarryBus transaction hung after Gazebo restart")
                self.assertTrue(
                    fresh_after_server_restart.get("ok"),
                    "fresh transaction failed after Gazebo restart")
                self.assertGreater(
                    fresh_after_server_restart.get("request_id", 0),
                    server_restart_result["request_id"])
            finally:
                if restarted_server_bridge is not None:
                    try:
                        os.killpg(restarted_server_bridge.pid, signal.SIGTERM)
                        restarted_server_bridge.wait(timeout=3.0)
                    except ProcessLookupError:
                        pass
                    except subprocess.TimeoutExpired:
                        os.killpg(restarted_server_bridge.pid, signal.SIGKILL)
                        restarted_server_bridge.wait(timeout=3.0)
                if restarted_sim.poll() is None:
                    restarted_sim.terminate()
                    try:
                        restarted_sim.wait(timeout=5.0)
                    except subprocess.TimeoutExpired:
                        restarted_sim.kill()
                        restarted_sim.wait(timeout=3.0)


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
