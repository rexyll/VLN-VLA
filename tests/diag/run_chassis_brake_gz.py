#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Isolated Gazebo Sim 8 wheel-brake hold / release experiment.

This launches tests/diag/chassis_brake_test.sdf only. It never loads the brake
into the production work_area world. A persistent external yaw wrench provides
a repeatable disturbance; the script compares DiffDrive zero-command behavior
against the opt-in JointForceCmd PD brake, then verifies DiffDrive recovers
after the brake component is removed.
"""
import argparse
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
from gz.msgs10.odometry_pb2 import Odometry
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.stringmsg_pb2 import StringMsg

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
WORLD = os.path.join(ROOT, "tests", "diag", "chassis_brake_test.sdf")
POSE_TOPIC = "/world/brake_test/dynamic_pose/info"
JOINT_TOPIC = "/world/brake_test/model/robot/joint_state"
STATUS_TOPIC = "/chassis_brake/status"
CMD_TOPIC = "/model/robot/cmd_vel"
ODOM_TOPIC = "/model/robot/odometry"
WRENCH_TOPIC = "/world/brake_test/wrench/persistent"
CLEAR_TOPIC = "/world/brake_test/wrench/clear"
BRAKE_TOPIC = "/chassis_brake/enable"
# Diagnostic guard only; not a physical acceptance threshold.
MAX_DIAGNOSTIC_WHEEL_RAD_S = 100.0


def yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Telemetry:
    def __init__(self):
        self.lock = threading.Lock()
        self.pose = None
        self.wheel_velocity = {}
        self.wheel_position = {}
        self.twist = None
        self.status = []
        self.node = gz_transport.Node()
        if not self.node.subscribe(Pose_V, POSE_TOPIC, self.on_pose):
            raise RuntimeError("cannot subscribe " + POSE_TOPIC)
        if not self.node.subscribe(Model, JOINT_TOPIC, self.on_joints):
            raise RuntimeError("cannot subscribe " + JOINT_TOPIC)
        if not self.node.subscribe(StringMsg, STATUS_TOPIC, self.on_status):
            raise RuntimeError("cannot subscribe " + STATUS_TOPIC)
        if not self.node.subscribe(Odometry, ODOM_TOPIC, self.on_odometry):
            raise RuntimeError("cannot subscribe " + ODOM_TOPIC)

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
        positions = {}
        for joint in msg.joint:
            if joint.name in ("left_wheel_joint", "right_wheel_joint"):
                if joint.HasField("axis1"):
                    positions[joint.name] = float(joint.axis1.position)
        with self.lock:
            self.wheel_velocity = values
            self.wheel_position = positions

    def on_odometry(self, msg):
        twist = (float(msg.twist.linear.x), float(msg.twist.linear.y),
                 float(msg.twist.angular.z))
        with self.lock:
            self.twist = twist

    def on_status(self, msg):
        with self.lock:
            self.status.append(msg.data)

    def snapshot(self):
        with self.lock:
            return self.pose, dict(self.wheel_velocity), list(self.status)

    def motion_snapshot(self):
        with self.lock:
            return (self.pose, self.twist, dict(self.wheel_position),
                    dict(self.wheel_velocity))


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


def ensure_sim_alive(process, phase):
    return_code = process.poll()
    if return_code is not None:
        raise RuntimeError(
            "Gazebo process exited during %s (exit_code=%s)" %
            (phase, return_code))


def wait_until(predicate, timeout, description, process=None):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if process is not None:
            ensure_sim_alive(process, description)
        if predicate():
            return
        time.sleep(0.02)
    if process is not None:
        ensure_sim_alive(process, description)
    raise RuntimeError("timeout waiting for " + description)


def measure_with_wrench(telemetry, duration, publisher, phase, log, process):
    """Publish the same persistent wrench throughout the measurement window."""
    start_pose, start_twist, start_positions, start_velocities = (
        telemetry.motion_snapshot())
    if start_pose is None:
        raise RuntimeError("robot pose not available")
    max_xy = 0.0
    max_yaw = 0.0
    max_wheel = 0.0
    publish_count = 0
    deadline = time.monotonic() + duration
    next_publish = time.monotonic()
    while time.monotonic() < deadline:
        ensure_sim_alive(process, phase + " wrench measurement")
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
        motion = telemetry.motion_snapshot()
        twist_values = motion[1] or (0.0, 0.0, 0.0)
        position_values = motion[2].values()
        velocity_values = motion[3].values()
        if not all(math.isfinite(v) for v in
                   (max_xy, max_yaw, max_wheel, *twist_values,
                    *position_values, *velocity_values)):
            raise RuntimeError("%s produced non-finite physical telemetry" % phase)
        if max_wheel > MAX_DIAGNOSTIC_WHEEL_RAD_S:
            log.write(
                "%s runaway_wheel_velocity_rad_s=%.6f limit=%.1f "
                "pose=%r wheels=%r\n" %
                (phase, max_wheel, MAX_DIAGNOSTIC_WHEEL_RAD_S,
                 pose, wheels))
            log.flush()
            raise RuntimeError(
                "%s unstable dynamics: wheel velocity %.3f rad/s "
                "exceeded diagnostic guard %.1f rad/s" %
                (phase, max_wheel, MAX_DIAGNOSTIC_WHEEL_RAD_S))
        time.sleep(0.01)
    log.write("%s_wrench_publish_count=%d duration_s=%.3f\n" %
              (phase, publish_count, duration))
    log.flush()
    if publish_count < 5:
        raise RuntimeError("%s wrench publisher delivered too few messages" % phase)
    end_pose = telemetry.snapshot()[0]
    log.write("%s_pose_start=%r pose_end=%r elapsed_s=%.3f\n" %
              (phase, start_pose, end_pose, duration))
    log.flush()
    return {"max_xy_mm": max_xy * 1000.0,
            "max_yaw_deg": max_yaw,
            "max_wheel_rad_s": max_wheel,
            "pose_start": start_pose, "pose_end": end_pose,
            "twist_start": start_twist, "twist_end": end_twist,
            "wheel_position_start": start_positions,
            "wheel_position_end": end_positions,
            "wheel_velocity_start": start_velocities,
            "wheel_velocity_end": end_velocities,
            "duration_s": duration}


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




def run_single(torque_nm, log_suffix=""):
    if not os.path.isfile(WORLD):
        raise RuntimeError("test world missing: " + WORLD)
    log_path = os.path.join(ROOT, "vla_ws", "logs", "chassis_brake_isolated%s.log" % log_suffix)
    runner_log_path = os.path.join(ROOT, "vla_ws", "logs", "chassis_brake_runner%s.log" % log_suffix)
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
                       "initial Gazebo robot pose", process)
            wait_until(lambda: len(telemetry.snapshot()[1]) == 2, 30,
                       "both wheel joint states", process)

            publish(CMD_TOPIC, "gz.msgs.Twist",
                    "linear: {x: 0.0} angular: {z: 0.0}")
            time.sleep(0.5)

            # Both publishers are long-lived Gazebo Transport objects. Repeated
            # messages span the complete measurement window so discovery startup
            # cannot consume the only disturbance sample.
            wait_for_subscriber(WRENCH_TOPIC, "gz.msgs.EntityWrench", 15,
                                runner_log)
            wrench = ('entity: {name: "chassis" type: LINK} '
                      'wrench: {torque: {z: %.9g}}' % torque_nm)
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
            try:
                baseline = measure_with_wrench(
                    telemetry, 1.0, wrench_pub, "baseline", runner_log, process)
            finally:
                publish_clear_repeated(clear_pub, "baseline", runner_log)
            record("candidate_torque_nm=%.9g baseline=%r" % (torque_nm, baseline))
            if baseline["max_yaw_deg"] < 0.5:
                raise RuntimeError(
                    "baseline disturbance too weak: yaw %.3f deg < 0.5 deg" %
                    baseline["max_yaw_deg"])
            time.sleep(0.5)
            ensure_sim_alive(process, "post-baseline settling")

            # Brake: latch current wheel angles, then apply the identical,
            # repeatedly published disturbance through the same publisher.
            publish(BRAKE_TOPIC, "gz.msgs.Boolean", "data: true")
            wait_until(lambda: any(s.startswith("LOCKED")
                                   for s in telemetry.snapshot()[2]),
                       5, "ChassisBrake LOCKED component acknowledgement",
                       process)
            try:
                locked = measure_with_wrench(
                    telemetry, 1.0, wrench_pub, "locked", runner_log, process)
            finally:
                publish_clear_repeated(clear_pub, "locked", runner_log)
            record("candidate_torque_nm=%.9g locked=%r" % (torque_nm, locked))
            time.sleep(0.25)

            publish(BRAKE_TOPIC, "gz.msgs.Boolean", "data: false")
            wait_until(lambda: any(s.startswith("UNLOCKED")
                                   for s in telemetry.snapshot()[2]),
                       5, "ChassisBrake UNLOCKED component removal",
                       process)

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
            log.write("gazebo_exit_code=%s\\n" % process.returncode)
            log.flush()


def main():
    parser = argparse.ArgumentParser(
        description="Fresh-process ChassisBrake torque calibration")
    parser.add_argument("--single-torque", type=float, default=None,
                        help="run one isolated Gazebo experiment at this torque")
    parser.add_argument("--log-suffix", default="",
                        help="suffix used to preserve per-candidate logs")
    parser.add_argument(
        "--torque-candidates", default="0.05,0.075,0.1,0.15,0.25,0.4,0.6,0.9",
        help="ascending comma-separated torque candidates in N m")
    args = parser.parse_args()

    if args.single_torque is not None:
        if not math.isfinite(args.single_torque) or not 0.0 < args.single_torque <= 18.0:
            raise ValueError("single torque must be finite and in (0, 18] N m")
        return run_single(args.single_torque, args.log_suffix)

    candidates = []
    for token in args.torque_candidates.split(","):
        value = float(token.strip())
        if not math.isfinite(value) or not 0.0 < value <= 18.0:
            raise ValueError("torque candidates must be finite and in (0, 18] N m")
        candidates.append(value)
    if not candidates or candidates != sorted(set(candidates)):
        raise ValueError("torque candidates must be non-empty, unique and ascending")

    log_dir = os.path.join(ROOT, "vla_ws", "logs")
    os.makedirs(log_dir, exist_ok=True)
    summary_path = os.path.join(log_dir, "chassis_brake_runner.log")
    with open(summary_path, "w", encoding="utf-8") as summary:
        summary.write("calibration_mode=fresh_gazebo_process_per_candidate\n")
        summary.write("torque_candidates_nm=%r\n" % candidates)
        summary.flush()
        for index, torque_nm in enumerate(candidates):
            suffix = "_torque_" + ("%.6g" % torque_nm).replace(".", "p")
            command = [
                sys.executable, os.path.abspath(__file__),
                "--single-torque", "%.9g" % torque_nm,
                "--log-suffix", suffix]
            started = time.monotonic()
            result = subprocess.run(
                command, cwd=ROOT, env=os.environ.copy(),
                text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
            elapsed = time.monotonic() - started
            summary.write(
                "candidate_index=%d torque_nm=%.9g exit_code=%d elapsed_s=%.3f\n"
                % (index, torque_nm, result.returncode, elapsed))
            summary.write(result.stdout)
            summary.flush()
            print(result.stdout, end="", flush=True)
            if result.returncode == 0 and "PASS: isolated brake hold" in result.stdout:
                summary.write("CALIBRATION_PASS torque_nm=%.9g\n" % torque_nm)
                summary.flush()
                print("CALIBRATION_PASS torque_nm=%.9g" % torque_nm, flush=True)
                return 0
            print("CALIBRATION_REJECT torque_nm=%.9g exit_code=%d" %
                  (torque_nm, result.returncode), flush=True)

    print("FAIL: no torque candidate satisfied baseline, locked-hold and unlock gates",
          file=sys.stderr)
    return 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:
        print("FAIL:", exc, file=sys.stderr)
        sys.exit(1)
