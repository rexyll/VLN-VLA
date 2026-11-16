#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Independent Gazebo Sim 8.15 Ground Fixed Joint attach/hold/release test.

This test intentionally excludes ApplyLinkWrench and ChassisBrake. It verifies
DetachableJoint state messages AND the robot's measured world pose response to
the same DiffDrive command while detached and attached.
"""
import argparse
import json
import math
import os
import signal
import subprocess
import sys
import threading
import time

import gz.transport13 as gz_transport
from gz.msgs10.stringmsg_pb2 import StringMsg
from gz.msgs10.empty_pb2 import Empty
from gz.msgs10.odometry_pb2 import Odometry
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.model_pb2 import Model

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
WORLD = os.path.join(ROOT, "tests", "diag", "chassis_ground_lock_test.sdf")
WORLD_NAME = "brake_test"
STATE_TOPIC = "/world/brake_test/chassis_lock/state"
ATTACH_TOPIC = "/world/brake_test/chassis_lock/attach"
DETACH_TOPIC = "/world/brake_test/chassis_lock/detach"
POSE_TOPIC = "/world/brake_test/dynamic_pose/info"
ODOM_TOPIC = "/model/robot/odometry"
JOINT_TOPIC = "/world/brake_test/model/robot/joint_state"
CMD_TOPIC = "/model/robot/cmd_vel"
COMMAND_SPEED = 0.12
COMMAND_WINDOW_S = 1.0
MAX_WHEEL_RAD_S = 100.0


def get_yaw(q):
    return math.atan2(2.0 * (q.w*q.z + q.x*q.y),
                      1.0 - 2.0 * (q.y*q.y + q.z*q.z))


class Telemetry:
    def __init__(self):
        self.lock = threading.Lock()
        self.pose = None
        self.twist = None
        self.wheel_velocity = {}
        self.joint_names = set()
        self.attached = None
        self.state_messages = []
        self.node = gz_transport.Node()
        for typ, topic, cb in (
            (Pose_V, POSE_TOPIC, self.on_pose),
            (Odometry, ODOM_TOPIC, self.on_odom),
            (Model, JOINT_TOPIC, self.on_joints),
            (StringMsg, STATE_TOPIC, self.on_state),
        ):
            if not self.node.subscribe(typ, topic, cb):
                raise RuntimeError("cannot subscribe " + topic)

    def on_pose(self, msg):
        for item in msg.pose:
            if item.name == "robot":
                p = item.position
                v = (float(p.x), float(p.y), float(p.z),
                     get_yaw(item.orientation))
                with self.lock:
                    self.pose = v
                return

    def on_odom(self, msg):
        t = msg.twist
        v = (float(t.linear.x), float(t.linear.y), float(t.angular.z))
        with self.lock:
            self.twist = v

    def on_joints(self, msg):
        speeds = {}
        names = set()
        for j in msg.joint:
            names.add(j.name)
            if j.name in ("left_wheel_joint", "right_wheel_joint") and j.HasField("axis1"):
                speeds[j.name] = float(j.axis1.velocity)
        with self.lock:
            self.wheel_velocity = speeds
            self.joint_names = names

    def on_state(self, msg):
        with self.lock:
            state = str(msg.data).strip().lower()
            if state not in ('attached', 'detached'):
                return
            self.attached = state
            self.state_messages.append(state)

    def snapshot(self):
        with self.lock:
            return (self.pose, self.twist, dict(self.wheel_velocity),
                    self.attached, list(self.state_messages),
                    set(self.joint_names))


def ensure_alive(proc, where):
    rc = proc.poll()
    if rc is not None:
        raise RuntimeError("Gazebo exited during %s (rc=%s)" % (where, rc))


def wait_for(predicate, timeout, description, proc):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        ensure_alive(proc, description)
        if predicate():
            return
        time.sleep(0.02)
    raise RuntimeError("timeout waiting for " + description)


def publish_empty(topic):
    r = subprocess.run(
        ["gz", "topic", "-t", topic, "-m", "gz.msgs.Empty", "-p", " "],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=10)
    if r.returncode:
        raise RuntimeError("Empty publish failed on %s: %s" % (topic, r.stdout))


def publish_velocity(linear):
    payload = "linear: {x: %.9g} angular: {z: 0.0}" % linear
    r = subprocess.run(
        ["gz", "topic", "-t", CMD_TOPIC, "-m", "gz.msgs.Twist", "-p", payload],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=10)
    if r.returncode:
        raise RuntimeError("Twist publish failed: " + r.stdout)


def wait_stationary(tel, proc, timeout=5.0):
    def stationary():
        pose, twist, wheels, _, _, _ = tel.snapshot()
        return (pose is not None and twist is not None and
                max(abs(x) for x in twist) < 0.005 and
                (not wheels or max(abs(x) for x in wheels.values()) < 0.05))
    wait_for(stationary, timeout, "robot stationary", proc)


def displacement(a, b):
    return math.sqrt(sum((b[i]-a[i])**2 for i in range(3)))


def run_once(index, output_dir):
    if not os.path.isfile(WORLD):
        raise RuntimeError("missing world: " + WORLD)
    os.makedirs(output_dir, exist_ok=True)
    log_path = os.path.join(output_dir, "ground_fixed_joint_%02d.log" % index)
    report_path = os.path.join(output_dir, "ground_fixed_joint_%02d.json" % index)
    report = {
        "schema_version": 1, "repeat": index,
        "world": os.path.relpath(WORLD, ROOT),
        "command_speed_m_s": COMMAND_SPEED,
        "command_window_s": COMMAND_WINDOW_S,
        "stages": [],
    }
    with open(log_path, "w", encoding="utf-8") as log:
        env = os.environ.copy()
        proc = subprocess.Popen(
            ["gz", "sim", "-s", "-r", WORLD], cwd=ROOT, env=env,
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            tel = Telemetry()
            wait_for(lambda: tel.snapshot()[0] is not None, 30,
                     "robot world pose", proc)
            wait_for(lambda: tel.snapshot()[1] is not None, 30,
                     "robot odometry", proc)
            wait_for(lambda: tel.snapshot()[3] is not None, 30,
                     "DetachableJoint initial state", proc)
            wait_for(lambda: len(tel.snapshot()[2]) == 2, 30,
                     "both wheel joint velocities", proc)
            initial = tel.snapshot()
            report["initial_pose"] = initial[0]
            report["initial_attached"] = initial[3]
            if initial[3] != 'attached':
                raise RuntimeError("Gazebo 8 DetachableJoint did not start attached")
            if max(abs(v) for v in initial[1]) > 0.01:
                raise RuntimeError("world_ready failed: chassis not stationary")

            # Stage 1: detach while stationary, then prove normal drive mobility.
            publish_empty(DETACH_TOPIC)
            wait_for(lambda: tel.snapshot()[3] == 'detached', 5,
                     "DetachableJoint detached state", proc)
            wait_stationary(tel, proc)
            free_start = tel.snapshot()[0]
            publish_velocity(COMMAND_SPEED)
            time.sleep(COMMAND_WINDOW_S)
            publish_velocity(0.0)
            wait_stationary(tel, proc, timeout=8.0)
            free_end = tel.snapshot()[0]
            free_mm = displacement(free_start, free_end) * 1000.0
            report["stages"].append({
                "name": "detached_drive", "attached": False,
                "displacement_mm": free_mm,
                "pose_start": free_start, "pose_end": free_end,
            })
            if free_mm < 10.0:
                raise RuntimeError("detached robot failed to move >=10mm")

            # Stage 2: attach at the current pose and challenge it with identical
            # drive commands. The Boolean ACK is necessary but not sufficient.
            attach_reference = tel.snapshot()[0]
            publish_empty(ATTACH_TOPIC)
            wait_for(lambda: tel.snapshot()[3] == 'attached', 5,
                     "DetachableJoint attached state", proc)
            wait_stationary(tel, proc)
            lock_start = tel.snapshot()[0]
            publish_velocity(COMMAND_SPEED)
            deadline = time.monotonic() + COMMAND_WINDOW_S
            max_wheel = 0.0
            while time.monotonic() < deadline:
                ensure_alive(proc, "locked drive challenge")
                wheels = tel.snapshot()[2]
                if wheels:
                    max_wheel = max(max_wheel, *(abs(v) for v in wheels.values()))
                if max_wheel > MAX_WHEEL_RAD_S:
                    raise RuntimeError("wheel speed exceeded diagnostic safety guard")
                time.sleep(0.02)
            publish_velocity(0.0)
            wait_stationary(tel, proc, timeout=8.0)
            lock_end = tel.snapshot()[0]
            lock_mm = displacement(lock_start, lock_end) * 1000.0
            yaw_delta = abs(math.degrees(math.atan2(
                math.sin(lock_end[3]-lock_start[3]),
                math.cos(lock_end[3]-lock_start[3]))))
            report["stages"].append({
                "name": "attached_drive_challenge", "attached": True,
                "displacement_mm": lock_mm, "yaw_delta_deg": yaw_delta,
                "max_wheel_rad_s": max_wheel,
                "pose_start": lock_start, "pose_end": lock_end,
            })
            if lock_mm >= 2.0 or yaw_delta >= 0.5:
                raise RuntimeError(
                    "ground fixed joint failed physical hold: %.3fmm, %.6fdeg" %
                    (lock_mm, yaw_delta))

            # Stage 3: detach again and prove drive recovery.
            publish_empty(DETACH_TOPIC)
            wait_for(lambda: tel.snapshot()[3] == 'detached', 5,
                     "DetachableJoint second detach", proc)
            wait_stationary(tel, proc)
            recovery_start = tel.snapshot()[0]
            publish_velocity(COMMAND_SPEED)
            time.sleep(COMMAND_WINDOW_S)
            publish_velocity(0.0)
            wait_stationary(tel, proc, timeout=8.0)
            recovery_end = tel.snapshot()[0]
            recovery_mm = displacement(recovery_start, recovery_end) * 1000.0
            report["stages"].append({
                "name": "unlock_recovery", "attached": False,
                "displacement_mm": recovery_mm,
                "pose_start": recovery_start, "pose_end": recovery_end,
            })
            if recovery_mm < 10.0:
                raise RuntimeError("unlock recovery displacement <10mm")

            report["final_attached"] = tel.snapshot()[3]
            report["state_messages"] = tel.snapshot()[4]
            report["result"] = "PASS"
            log.write("\nGROUND_FIXED_JOINT_PASS repeat=%d report=%s\n" %
                      (index, report_path))
        except Exception as exc:
            report["result"] = "FAIL"
            report["error"] = str(exc)
            report["state_messages"] = tel.snapshot()[4]
            report["last_reported_joint_state"] = tel.snapshot()[3]
            log.write("\nGROUND_FIXED_JOINT_FAIL repeat=%d error=%s\n" %
                      (index, exc))
            raise
        finally:
            try:
                publish_velocity(0.0)
            except Exception:
                pass
            try:
                os.killpg(proc.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(proc.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                proc.wait(timeout=5)
            report["gazebo_exit_code"] = proc.returncode
            log.flush()
            with open(report_path, "w", encoding="utf-8") as out:
                json.dump(report, out, indent=2, sort_keys=True)
                out.write("\n")
    return report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--output-dir", default=os.path.join(ROOT, "vla_ws", "logs"))
    args = parser.parse_args()
    if args.repeats < 1 or args.repeats > 10:
        raise ValueError("repeats must be between 1 and 10")
    results = []
    for i in range(1, args.repeats + 1):
        result = run_once(i, args.output_dir)
        results.append(result)
        print("GROUND_FIXED_JOINT_REPEAT_%d=PASS" % i, flush=True)
    summary = {
        "schema_version": 1, "repeats": args.repeats,
        "passed": sum(x.get("result") == "PASS" for x in results),
        "results": results,
    }
    path = os.path.join(args.output_dir, "ground_fixed_joint_summary.json")
    with open(path, "w", encoding="utf-8") as out:
        json.dump(summary, out, indent=2, sort_keys=True)
        out.write("\n")
    if summary["passed"] != args.repeats:
        raise RuntimeError("Ground Fixed Joint repeat acceptance failed")
    print("GROUND_FIXED_JOINT_ALL_PASS=%d" % args.repeats, flush=True)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:
        print("GROUND_FIXED_JOINT_FAIL:", exc, file=sys.stderr)
        sys.exit(1)
