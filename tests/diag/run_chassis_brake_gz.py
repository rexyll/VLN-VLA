#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Isolated Gazebo Sim 8 wheel-brake hold / release experiment.

This launches tests/diag/chassis_brake_test.sdf only. It never loads the brake
into the production work_area world. A persistent external yaw wrench provides
a repeatable disturbance; the script compares DiffDrive zero-command behavior
against the opt-in JointForceCmd PD brake, then verifies DiffDrive recovers
after the brake component is removed.
"""
import math
import os
import re
import signal
import subprocess
import sys
import threading
import time

import gz.transport13 as gz_transport
from google.protobuf import text_format
from gz.msgs10.entity_pb2 import Entity
from gz.msgs10.entity_wrench_pb2 import EntityWrench
from gz.msgs10.model_pb2 import Model
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.stringmsg_pb2 import StringMsg

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
WORLD = os.path.join(ROOT, "tests", "diag", "chassis_brake_test.sdf")
POSE_TOPIC = "/world/brake_test/dynamic_pose/info"
JOINT_TOPIC = "/world/brake_test/model/robot/joint_state"
STATUS_TOPIC = "/chassis_brake/status"
CMD_TOPIC = "/model/robot/cmd_vel"
WRENCH_TOPIC = "/world/brake_test/wrench/persistent"
CLEAR_TOPIC = "/world/brake_test/wrench/clear"
BRAKE_TOPIC = "/chassis_brake/enable"


def yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Telemetry:
    def __init__(self):
        self.lock = threading.Lock()
        self.pose = None
        self.wheel_velocity = {}
        self.status = []
        self.node = gz_transport.Node()
        if not self.node.subscribe(Pose_V, POSE_TOPIC, self.on_pose):
            raise RuntimeError("cannot subscribe " + POSE_TOPIC)
        if not self.node.subscribe(Model, JOINT_TOPIC, self.on_joints):
            raise RuntimeError("cannot subscribe " + JOINT_TOPIC)
        if not self.node.subscribe(StringMsg, STATUS_TOPIC, self.on_status):
            raise RuntimeError("cannot subscribe " + STATUS_TOPIC)

    def on_pose(self, msg):
        for item in msg.pose:
            if item.name == "robot":
                value = (float(item.position.x), float(item.position.y),
                         yaw(item.orientation))
                with self.lock:
                    self.pose = value
                return

    def on_joints(self, msg):
        values = {}
        for joint in msg.joint:
            if joint.name in ("left_wheel_joint", "right_wheel_joint"):
                if joint.HasField("axis1"):
                    # Axis.velocity is a proto3 scalar (no presence API).
                    values[joint.name] = float(joint.axis1.velocity)
        with self.lock:
            self.wheel_velocity = values

    def on_status(self, msg):
        with self.lock:
            self.status.append(msg.data)

    def snapshot(self):
        with self.lock:
            return self.pose, dict(self.wheel_velocity), list(self.status)


def publish(topic, msg_type, payload):
    result = subprocess.run(
        ["gz", "topic", "-t", topic, "-m", msg_type, "-p", payload],
        text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        timeout=10)
    if result.returncode:
        raise RuntimeError("gz topic publish failed: " + result.stdout)
    return result.stdout


class PersistentPublisher:
    """Keep a Gazebo Transport publisher alive across repeated deliveries."""

    def __init__(self, topic, message_type, payload):
        self.topic = topic
        self.node = gz_transport.Node()
        self.publisher = self.node.advertise(topic, message_type)
        if not self.publisher:
            raise RuntimeError("cannot advertise Gazebo topic " + topic)
        self.message = message_type()
        text_format.Parse(payload, self.message)

    def publish_once(self):
        if not self.publisher.publish(self.message):
            raise RuntimeError("Gazebo Transport publish returned false on " +
                               self.topic)


def topic_info(topic):
    result = subprocess.run(
        ["gz", "topic", "-i", "-t", topic],
        text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        timeout=10)
    if result.returncode:
        return result.stdout
    return result.stdout


def wait_for_subscriber(topic, message_type, timeout, log):
    """Require the ApplyLinkWrench subscriber before advertising the publisher."""
    deadline = time.monotonic() + timeout
    last_info = ""
    while time.monotonic() < deadline:
        last_info = topic_info(topic)
        # gz topic -i prints endpoint rows below the Subscribers heading.
        # Requiring the expected message type avoids mistaking a publisher
        # endpoint or an unrelated subscriber for ApplyLinkWrench readiness.
        sections = re.split(r"Subscribers\s*\[[^]]*\]\s*:", last_info,
                            maxsplit=1, flags=re.IGNORECASE)
        if len(sections) == 2:
            subscriber_section = sections[1].split("\n\n", 1)[0]
            if message_type in subscriber_section and "tcp://" in subscriber_section:
                log.write("subscriber_ready topic=%s type=%s\n%s\n" %
                          (topic, message_type, last_info))
                log.flush()
                return
        time.sleep(0.1)
    log.write("subscriber_timeout topic=%s type=%s\n%s\n" %
              (topic, message_type, last_info))
    log.flush()
    raise RuntimeError("no %s subscriber discovered for %s" %
                       (message_type, topic))


def wait_for_publisher(topic, message_type, timeout, log):
    """Confirm our long-lived publisher is visible to Gazebo discovery."""
    deadline = time.monotonic() + timeout
    last_info = ""
    while time.monotonic() < deadline:
        last_info = topic_info(topic)
        sections = re.split(r"Publishers\s*\[[^]]*\]\s*:", last_info,
                            maxsplit=1, flags=re.IGNORECASE)
        if len(sections) == 2:
            publisher_section = sections[1].split("\n\n", 1)[0]
            if message_type in publisher_section and "tcp://" in publisher_section:
                log.write("publisher_visible topic=%s type=%s\n%s\n" %
                          (topic, message_type, last_info))
                log.flush()
                return
        time.sleep(0.1)
    log.write("publisher_visibility_timeout topic=%s type=%s\n%s\n" %
              (topic, message_type, last_info))
    log.flush()
    raise RuntimeError("Gazebo publisher not discovered for " + topic)


def wait_until(predicate, timeout, description):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.02)
    raise RuntimeError("timeout waiting for " + description)


def measure_with_wrench(telemetry, duration, publisher, phase, log):
    """Publish the same persistent wrench throughout the measurement window."""
    start_pose, _, _ = telemetry.snapshot()
    if start_pose is None:
        raise RuntimeError("robot pose not available")
    max_xy = 0.0
    max_yaw = 0.0
    max_wheel = 0.0
    publish_count = 0
    deadline = time.monotonic() + duration
    next_publish = time.monotonic()
    while time.monotonic() < deadline:
        now = time.monotonic()
        if now >= next_publish:
            publisher.publish_once()
            publish_count += 1
            next_publish = now + 0.05
        pose, wheels, _ = telemetry.snapshot()
        if pose is not None:
            dx = pose[0] - start_pose[0]
            dy = pose[1] - start_pose[1]
            dyaw = math.atan2(math.sin(pose[2] - start_pose[2]),
                              math.cos(pose[2] - start_pose[2]))
            max_xy = max(max_xy, math.hypot(dx, dy))
            max_yaw = max(max_yaw, abs(math.degrees(dyaw)))
        if wheels:
            max_wheel = max(max_wheel, *(abs(v) for v in wheels.values()))
        time.sleep(0.01)
    log.write("%s_wrench_publish_count=%d duration_s=%.3f\n" %
              (phase, publish_count, duration))
    log.flush()
    if publish_count < 5:
        raise RuntimeError("%s wrench publisher delivered too few messages" % phase)
    return {"max_xy_mm": max_xy * 1000.0,
            "max_yaw_deg": max_yaw,
            "max_wheel_rad_s": max_wheel}


def publish_clear_repeated(publisher, phase, log):
    count = 0
    deadline = time.monotonic() + 0.25
    next_publish = time.monotonic()
    while time.monotonic() < deadline:
        publisher.publish_once()
        count += 1
        next_publish += 0.05
        time.sleep(max(0.0, next_publish - time.monotonic()))
    log.write("%s_wrench_clear_publish_count=%d entity=LINK:chassis\n" %
              (phase, count))
    log.flush()
    if count < 3:
        raise RuntimeError("%s wrench clear published too few messages" % phase)




def main():
    if not os.path.isfile(WORLD):
        raise RuntimeError("test world missing: " + WORLD)
    log_path = os.path.join(ROOT, "vla_ws", "logs", "chassis_brake_isolated.log")
    runner_log_path = os.path.join(ROOT, "vla_ws", "logs", "chassis_brake_runner.log")
    os.makedirs(os.path.dirname(log_path), exist_ok=True)
    with open(log_path, "w", encoding="utf-8") as log, \
         open(runner_log_path, "w", encoding="utf-8") as runner_log:
        def record(message):
            print(message, flush=True)
            runner_log.write(message + "\n")
            runner_log.flush()

        env = os.environ.copy()
        plugin_dir = os.path.join(
            ROOT, "vla_ws", "src", "vla_follow", "gz_plugins", "build")
        env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = (
            plugin_dir + ":" + env.get("GZ_SIM_SYSTEM_PLUGIN_PATH", ""))
        process = subprocess.Popen(
            ["gz", "sim", "-s", "-r", WORLD],
            cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT,
            start_new_session=True)
        try:
            telemetry = Telemetry()
            wait_until(lambda: telemetry.snapshot()[0] is not None, 30,
                       "initial Gazebo robot pose")
            wait_until(lambda: len(telemetry.snapshot()[1]) == 2, 30,
                       "both wheel joint states")

            publish(CMD_TOPIC, "gz.msgs.Twist",
                    "linear: {x: 0.0} angular: {z: 0.0}")
            time.sleep(0.5)

            # Both publishers are long-lived Gazebo Transport objects. Repeated
            # messages span the complete measurement window so discovery startup
            # cannot consume the only disturbance sample.
            wait_for_subscriber(WRENCH_TOPIC, "gz.msgs.EntityWrench", 15,
                                runner_log)
            wrench = ('entity: {name: "chassis" type: LINK} '
                      'wrench: {torque: {z: 18.0}}')
            clear = 'name: "chassis" type: LINK'
            wrench_pub = PersistentPublisher(WRENCH_TOPIC, EntityWrench, wrench)
            wait_for_subscriber(CLEAR_TOPIC, "gz.msgs.Entity", 15,
                                runner_log)
            clear_pub = PersistentPublisher(CLEAR_TOPIC, Entity, clear)
            wait_for_publisher(WRENCH_TOPIC, "gz.msgs.EntityWrench", 15,
                               runner_log)
            wait_for_publisher(CLEAR_TOPIC, "gz.msgs.Entity", 15,
                               runner_log)

            # Baseline: the physical response gate remains the calibration
            # proof that ApplyLinkWrench actually affected the chassis.
            baseline = measure_with_wrench(
                telemetry, 1.0, wrench_pub, "baseline", runner_log)
            record("baseline=" + repr(baseline))
            publish_clear_repeated(clear_pub, "baseline", runner_log)
            time.sleep(0.5)

            # Brake: latch current wheel angles, then apply the identical,
            # repeatedly published disturbance through the same publisher.
            publish(BRAKE_TOPIC, "gz.msgs.Boolean", "data: true")
            wait_until(lambda: any(s.startswith("LOCKED")
                                   for s in telemetry.snapshot()[2]),
                       5, "ChassisBrake LOCKED component acknowledgement")
            locked = measure_with_wrench(
                telemetry, 1.0, wrench_pub, "locked", runner_log)
            record("locked=" + repr(locked))
            publish_clear_repeated(clear_pub, "locked", runner_log)
            time.sleep(0.25)

            publish(BRAKE_TOPIC, "gz.msgs.Boolean", "data: false")
            wait_until(lambda: any(s.startswith("UNLOCKED")
                                   for s in telemetry.snapshot()[2]),
                       5, "ChassisBrake UNLOCKED component removal")

            # Release recovery: ordinary DiffDrive command must move the base.
            drive_start = telemetry.snapshot()[0]
            publish(CMD_TOPIC, "gz.msgs.Twist",
                    "linear: {x: 0.12} angular: {z: 0.0}")
            time.sleep(0.8)
            publish(CMD_TOPIC, "gz.msgs.Twist",
                    "linear: {x: 0.0} angular: {z: 0.0}")
            drive_end = telemetry.snapshot()[0]
            recovery_mm = 0.0
            if drive_start is not None and drive_end is not None:
                recovery_mm = math.hypot(drive_end[0] - drive_start[0],
                                         drive_end[1] - drive_start[1]) * 1000.0

            record("unlock_recovery_displacement_mm=" + str(round(recovery_mm, 3)))

            # First ensure the disturbance actually challenged the baseline.
            if baseline["max_yaw_deg"] < 0.5:
                raise RuntimeError(
                    "baseline disturbance too weak to distinguish brake hold")
            if locked["max_yaw_deg"] >= baseline["max_yaw_deg"] * 0.5:
                raise RuntimeError(
                    "brake did not reduce yaw drift by at least 50 percent")
            if recovery_mm < 10.0:
                raise RuntimeError(
                    "DiffDrive did not recover after JointForceCmd removal")
            record("PASS: isolated brake hold and unlock recovery")
            return 0
        finally:
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                process.wait(timeout=5)


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:
        print("FAIL:", exc, file=sys.stderr)
        sys.exit(1)
