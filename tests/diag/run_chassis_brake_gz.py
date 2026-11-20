#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Isolated Gazebo Sim 8 wheel-brake hold / release experiment.

This launches tests/diag/chassis_brake_test.sdf only. It never loads the brake
into the production work_area world. A repeated nonzero DiffDrive command is
the disturbance: baseline motion must be physically observed, the opt-in
JointForceCmd PD brake must resist the same command, and DiffDrive must recover
after the brake component is removed.
"""
import argparse
import hashlib
import json
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
WRENCH_ENTITY_NAME = "robot"
WRENCH_ENTITY_TYPE = "MODEL"
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
        self.model_entity_id = None
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
                    self.model_entity_id = int(item.id)
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

    def entity_id(self):
        with self.lock:
            return self.model_entity_id

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


def measure_with_drive_command(telemetry, duration, phase, log, process):
    """Measure chassis response while repeatedly commanding DiffDrive motion."""
    command = "linear: {x: 0.12} angular: {z: 0.15}"
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
    try:
        while time.monotonic() < deadline:
            ensure_sim_alive(process, phase + " drive-command measurement")
            now = time.monotonic()
            if now >= next_publish:
                publish(CMD_TOPIC, "gz.msgs.Twist", command)
                publish_count += 1
                next_publish = now + 0.05
            pose, _, _ = telemetry.snapshot()
            motion = telemetry.motion_snapshot()
            if pose is not None:
                dx = pose[0] - start_pose[0]
                dy = pose[1] - start_pose[1]
                dyaw = math.atan2(math.sin(pose[2] - start_pose[2]),
                                  math.cos(pose[2] - start_pose[2]))
                max_xy = max(max_xy, math.hypot(dx, dy))
                max_yaw = max(max_yaw, abs(math.degrees(dyaw)))
            wheel_values = motion[3] or {}
            if wheel_values:
                max_wheel = max(max_wheel, *(abs(v) for v in wheel_values.values()))
            twist_values = motion[1] or (0.0, 0.0, 0.0)
            if not all(math.isfinite(v) for v in
                       (max_xy, max_yaw, max_wheel, *twist_values,
                        *(motion[2] or {}).values(), *wheel_values.values())):
                raise RuntimeError("%s produced non-finite physical telemetry" % phase)
            if max_wheel > MAX_DIAGNOSTIC_WHEEL_RAD_S:
                raise RuntimeError(
                    "%s unstable dynamics: wheel velocity %.3f rad/s exceeded %.1f" %
                    (phase, max_wheel, MAX_DIAGNOSTIC_WHEEL_RAD_S))
            time.sleep(0.01)
    finally:
        publish(CMD_TOPIC, "gz.msgs.Twist",
                "linear: {x: 0.0} angular: {z: 0.0}")
    log.write("%s_drive_command_publish_count=%d duration_s=%.3f\\n" %
              (phase, publish_count, duration))
    log.flush()
    if publish_count < 5:
        raise RuntimeError("%s drive command was published too few times" % phase)
    end_pose, end_twist, end_positions, end_velocities = (
        telemetry.motion_snapshot())
    result = {
        "max_xy_mm": max_xy * 1000.0,
        "max_yaw_deg": max_yaw,
        "max_wheel_rad_s": max_wheel,
        "pose_start": start_pose, "pose_end": end_pose,
        "twist_start": start_twist, "twist_end": end_twist,
        "wheel_position_start": start_positions,
        "wheel_position_end": end_positions,
        "wheel_velocity_start": start_velocities,
        "wheel_velocity_end": end_velocities,
        "duration_s": duration,
    }
    log.write("%s_drive_result=%r\\n" % (phase, result))
    log.flush()
    return result


def stage_lock_inputs():
    plugin_path = os.path.join(
        ROOT, "vla_ws", "src", "vla_follow", "gz_plugins", "build",
        "libChassisBrake.so")
    plugin_source = os.path.join(
        ROOT, "vla_ws", "src", "vla_follow", "gz_plugins", "ChassisBrake.cc")
    return {
        "world": WORLD,
        "runner": os.path.abspath(__file__),
        "plugin_source": plugin_source,
        "plugin_binary": plugin_path,
        "stage_verifier": os.path.join(
            ROOT, "tests", "diag", "verify_p104_stage_locks.py"),
    }


def sha256_file(path):
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def lock_inputs():
    return {name: {"path": os.path.relpath(path, ROOT),
                   "sha256": sha256_file(path)}
            for name, path in stage_lock_inputs().items()}


def write_stage_lock(path, stage, torque_nm, metrics, inputs):
    if lock_inputs() != inputs:
        raise RuntimeError("stage lock inputs changed before stage commit")
    try:
        with open(path, encoding="utf-8") as stream:
            manifest = json.load(stream)
    except FileNotFoundError:
        manifest = {
            "schema_version": 1,
            "git_sha": os.environ.get("GITHUB_SHA", "local"),
            "candidate_torque_nm": torque_nm,
            "input_hashes": inputs,
            "stages": [],
        }
    if manifest["input_hashes"] != inputs:
        raise RuntimeError("stage lock input hashes changed during experiment")
    if manifest["candidate_torque_nm"] != torque_nm:
        raise RuntimeError("stage lock candidate torque mismatch")
    expected = ["world_ready", "baseline_disturbance",
                "locked_hold", "unlock_recovery"]
    next_index = len(manifest["stages"])
    if next_index >= len(expected) or expected[next_index] != stage:
        raise RuntimeError("out-of-order stage lock: %s at index %d" %
                           (stage, next_index))
    manifest["stages"].append({
        "name": stage,
        "status": "passed",
        "metrics": metrics,
    })
    temporary = path + ".tmp"
    with open(temporary, "w", encoding="utf-8") as stream:
        json.dump(manifest, stream, indent=2, sort_keys=True)
        stream.write("\n")
    os.replace(temporary, path)
    return manifest


def verify_stage_lock(path, expected_stages, inputs, torque_nm):
    if lock_inputs() != inputs:
        raise RuntimeError("stage lock inputs changed between physical stages")
    with open(path, encoding="utf-8") as stream:
        manifest = json.load(stream)
    actual = [stage.get("name") for stage in manifest.get("stages", [])]
    if actual != expected_stages:
        raise RuntimeError("stage lock sequence mismatch: %r" % actual)
    if any(stage.get("status") != "passed"
           for stage in manifest.get("stages", [])):
        raise RuntimeError("stage lock contains a non-passed stage")
    if manifest.get("input_hashes") != inputs:
        raise RuntimeError("stage lock source/binary checksum mismatch")
    if manifest.get("candidate_torque_nm") != torque_nm:
        raise RuntimeError("stage lock torque mismatch")
    current_sha = os.environ.get("GITHUB_SHA")
    if current_sha and manifest.get("git_sha") != current_sha:
        raise RuntimeError("stage lock belongs to a different Git commit")
    return manifest


def publish_clear_repeated(publisher, phase, log):
    count = 0
    deadline = time.monotonic() + 0.25
    next_publish = time.monotonic()
    while time.monotonic() < deadline:
        publisher.publish_once()
        count += 1
        next_publish += 0.05
        time.sleep(max(0.0, next_publish - time.monotonic()))
    log.write("%s_wrench_clear_publish_count=%d entity=MODEL:robot\n" %
              (phase, count))
    log.flush()
    if count < 3:
        raise RuntimeError("%s wrench clear published too few messages" % phase)




def run_single(torque_nm, log_suffix=""):
    if not os.path.isfile(WORLD):
        raise RuntimeError("test world missing: " + WORLD)
    log_path = os.path.join(ROOT, "vla_ws", "logs", "chassis_brake_isolated%s.log" % log_suffix)
    runner_log_path = os.path.join(ROOT, "vla_ws", "logs", "chassis_brake_runner%s.log" % log_suffix)
    stage_lock_path = os.path.join(
        ROOT, "vla_ws", "logs", "p104_stage_locks%s.json" % log_suffix)
    os.makedirs(os.path.dirname(log_path), exist_ok=True)
    if os.path.exists(stage_lock_path):
        os.remove(stage_lock_path)
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
            wait_until(lambda: telemetry.entity_id() not in (None, 0), 30,
                       "robot model entity ID", process)
            wait_until(lambda: len(telemetry.snapshot()[1]) == 2, 30,
                       "both wheel joint states", process)
            wait_until(lambda: telemetry.motion_snapshot()[1] is not None, 30,
                       "DiffDrive odometry twist", process)
            wait_until(lambda: len(telemetry.motion_snapshot()[2]) == 2, 30,
                       "both wheel joint positions", process)
            inputs = lock_inputs()
            initial = telemetry.motion_snapshot()
            initial_twist = initial[1] or (math.inf, math.inf, math.inf)
            if (not all(math.isfinite(v) for v in
                        (*initial[0], *initial_twist,
                         *initial[2].values(), *initial[3].values())) or
                    max(abs(v) for v in initial_twist) > 0.01 or
                    max(abs(v) for v in initial[3].values()) > 0.01):
                raise RuntimeError(
                    "world_ready gate failed: initial chassis is not stationary")
            write_stage_lock(
                stage_lock_path, "world_ready", torque_nm,
                {"pose": initial[0], "model_entity_id": telemetry.entity_id(),
                 "twist": initial_twist,
                 "wheel_positions": initial[2],
                 "wheel_velocities": initial[3]}, inputs)
            verify_stage_lock(stage_lock_path, ["world_ready"], inputs,
                              torque_nm)

            publish(CMD_TOPIC, "gz.msgs.Twist",
                    "linear: {x: 0.0} angular: {z: 0.0}")
            time.sleep(0.5)

            # Calibrate with a real drive command, not transport delivery to
            # ApplyLinkWrench. This proves the base can move before the brake
            # is asked to resist the same command.
            baseline = measure_with_drive_command(
                telemetry, 1.0, "baseline", runner_log, process)
            record("candidate_torque_nm=%.9g baseline=%r" % (torque_nm, baseline))
            if baseline["max_yaw_deg"] < 0.5 and baseline["max_xy_mm"] < 10.0:
                raise RuntimeError(
                    "baseline DiffDrive disturbance not physically observed: "
                    "%.3fmm / %.6fdeg" %
                    (baseline["max_xy_mm"], baseline["max_yaw_deg"]))
            write_stage_lock(
                stage_lock_path, "baseline_disturbance", torque_nm,
                baseline, inputs)
            verify_stage_lock(
                stage_lock_path,
                ["world_ready", "baseline_disturbance"], inputs, torque_nm)
            time.sleep(0.5)
            ensure_sim_alive(process, "post-baseline settling")

            # Brake: latch current wheel angles, then issue the exact same
            # DiffDrive disturbance while the wheel-force hold is active.
            publish(BRAKE_TOPIC, "gz.msgs.Boolean", "data: true")
            wait_until(lambda: any(s.startswith("LOCKED")
                                   for s in telemetry.snapshot()[2]),
                       5, "ChassisBrake LOCKED component acknowledgement",
                       process)
            locked = measure_with_drive_command(
                telemetry, 1.0, "locked", runner_log, process)
            record("candidate_torque_nm=%.9g locked=%r" % (torque_nm, locked))
            if locked["max_yaw_deg"] >= baseline["max_yaw_deg"] * 0.5:
                raise RuntimeError(
                    "brake did not reduce yaw drift by at least 50 percent")
            write_stage_lock(
                stage_lock_path, "locked_hold", torque_nm,
                {"baseline_yaw_deg": baseline["max_yaw_deg"],
                 "locked_yaw_deg": locked["max_yaw_deg"],
                 "reduction_ratio": 1.0 - locked["max_yaw_deg"] /
                                    baseline["max_yaw_deg"],
                 "locked": locked}, inputs)
            verify_stage_lock(
                stage_lock_path,
                ["world_ready", "baseline_disturbance", "locked_hold"],
                inputs, torque_nm)
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
            if recovery_mm < 10.0:
                raise RuntimeError(
                    "DiffDrive did not recover after JointForceCmd removal")
            write_stage_lock(
                stage_lock_path, "unlock_recovery", torque_nm,
                {"displacement_mm": recovery_mm}, inputs)
            manifest = verify_stage_lock(
                stage_lock_path,
                ["world_ready", "baseline_disturbance",
                 "locked_hold", "unlock_recovery"], inputs, torque_nm)
            canonical_lock = os.path.join(
                ROOT, "vla_ws", "logs", "p104_chassis_brake_stage_locks.json")
            temporary_lock = canonical_lock + ".tmp"
            with open(temporary_lock, "w", encoding="utf-8") as stream:
                json.dump(manifest, stream, indent=2, sort_keys=True)
                stream.write("\n")
            os.replace(temporary_lock, canonical_lock)
            record("PASS: isolated brake hold and unlock recovery")
            record("STAGE_LOCKS_VERIFIED=" + canonical_lock)
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
            log.write("gazebo_exit_code=%s\n" % process.returncode)
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
    # Never allow a previous successful run to satisfy a later failed run.
    canonical_lock = os.path.join(
        log_dir, "p104_chassis_brake_stage_locks.json")
    if os.path.exists(canonical_lock):
        os.remove(canonical_lock)
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
