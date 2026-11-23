#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Full production robot model probe with chassis fixed directly to world.

Creates a diagnostic copy of work_area.sdf, removes only the external
DetachableJoint anchor and GraspCarry world plugin, and inserts a world-fixed
joint into robot. Production files are never modified.
"""
import json
import math
import os
import re
import signal
import subprocess
import threading
import time

import gz.transport13 as gz_transport
from gz.msgs10.model_pb2 import Model
from gz.msgs10.pose_v_pb2 import Pose_V

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
SOURCE = os.path.join(ROOT, "vla_ws", "src", "vla_follow", "worlds", "work_area.sdf")
LOG_DIR = os.path.join(ROOT, "vla_ws", "logs")
WORLD_OUT = os.path.join(LOG_DIR, "full_robot_world_fixed.sdf")
LOG_OUT = os.path.join(LOG_DIR, "full_robot_world_fixed_gz.log")
REPORT_OUT = os.path.join(LOG_DIR, "full_robot_world_fixed.json")
JOINT_TOPIC = "/world/follow_world/model/robot/joint_state"
POSE_TOPIC = "/world/follow_world/dynamic_pose/info"
BASE_CMD_TOPIC = "/model/robot/cmd_vel"
ARM_JOINTS = {
    "R_lift_joint": -0.15,
    "R_elbow_joint": -0.15,
    "R_wrist_p_joint": 0.15,
    "L_lift_joint": -0.15,
    "L_elbow_joint": -0.15,
    "L_wrist_p_joint": 0.15,
}
MAX_BASE_DISPLACEMENT_M = 0.002
MAX_BASE_YAW_RAD = math.radians(0.5)


def make_world():
    with open(SOURCE, encoding="utf-8") as stream:
        source = stream.read()
    anchor = re.compile(
        r"\s*<!-- Physical chassis immobilizer[\s\S]*?</model>\s*", re.MULTILINE
    )
    source, removed = anchor.subn("\n", source, count=1)
    if removed != 1:
        raise RuntimeError("expected exactly one production DetachableJoint anchor block")
    # The diagnostic does not exercise grasping; remove the optional custom
    # world plugin so this probe depends only on standard Gazebo systems.
    source, grasp_removed = re.subn(
        r"\s*<plugin filename=\"libGraspCarry\.so\" name=\"vla::GraspCarry\">"
        r"[\s\S]*?</plugin>\s*", "\n", source, count=1
    )
    if grasp_removed != 1:
        raise RuntimeError("expected exactly one GraspCarry world plugin")
    robot = re.search(r"<model name=\"robot\" canonical_link=\"chassis\">", source)
    if not robot:
        raise RuntimeError("production robot model not found")
    robot_end = source.find("</model>", robot.start())
    if robot_end < 0:
        raise RuntimeError("production robot model closing tag not found")
    fixed_joint = """
      <!-- Diagnostic-only: preserve robot internal DOFs while chassis is world-fixed. -->
      <joint name="world_fixed_chassis" type="fixed">
        <parent>world</parent>
        <child>chassis</child>
      </joint>
"""
    source = source[:robot_end] + fixed_joint + source[robot_end:]
    with open(WORLD_OUT, "w", encoding="utf-8") as stream:
        stream.write(source)
    check = subprocess.run(
        ["gz", "sdf", "-k", WORLD_OUT], cwd=ROOT, text=True,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=30
    )
    if check.returncode:
        raise RuntimeError("generated full robot SDF invalid:\n" + check.stdout)
    return {
        "source_sdf": os.path.relpath(SOURCE, ROOT),
        "diagnostic_sdf": os.path.relpath(WORLD_OUT, ROOT),
        "external_detachable_joint_removed": True,
        "grasp_carry_world_plugin_removed": True,
        "world_fixed_joint_inserted": True,
        "sdf_check": check.stdout[-3000:],
    }


class Telemetry:
    def __init__(self):
        self.lock = threading.Lock()
        self.joints = {}
        self.pose = None
        self.pose_samples = []
        self.node = gz_transport.Node()
        if not self.node.subscribe(Model, JOINT_TOPIC, self.on_joints):
            raise RuntimeError("cannot subscribe " + JOINT_TOPIC)
        if not self.node.subscribe(Pose_V, POSE_TOPIC, self.on_pose):
            raise RuntimeError("cannot subscribe " + POSE_TOPIC)

    def on_joints(self, msg):
        values = {}
        for joint in msg.joint:
            if joint.name in ARM_JOINTS and joint.HasField("axis1"):
                values[joint.name] = {
                    "position": float(joint.axis1.position),
                    "velocity": float(joint.axis1.velocity),
                    "effort": float(joint.axis1.force),
                }
        with self.lock:
            self.joints.update(values)

    def on_pose(self, msg):
        for item in msg.pose:
            if item.name == "robot":
                p, q = item.position, item.orientation
                yaw = math.atan2(
                    2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z),
                )
                value = (float(p.x), float(p.y), float(p.z), yaw)
                with self.lock:
                    self.pose = value
                    self.pose_samples.append((time.monotonic(), value))
                return

    def snapshot(self):
        with self.lock:
            return dict(self.joints), self.pose, list(self.pose_samples)


def publish(topic, msg_type, payload):
    result = subprocess.run(
        ["gz", "topic", "-t", topic, "-m", msg_type, "-p", payload],
        cwd=ROOT, text=True, stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT, timeout=10,
    )
    if result.returncode:
        raise RuntimeError("publish failed %s: %s" % (topic, result.stdout))


def wait_ready(tel, process, timeout=30.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError("Gazebo exited before telemetry became ready")
        joints, pose, _ = tel.snapshot()
        if len(joints) == len(ARM_JOINTS) and pose is not None:
            return
        time.sleep(0.05)
    joints, pose, _ = tel.snapshot()
    raise RuntimeError("telemetry timeout; joints=%s pose=%r" % (sorted(joints), pose))


def move_joint(tel, process, name, target):
    topic = "/drink/cmd/" + name
    payload = "data: %.6f" % target
    initial = tel.snapshot()[0].get(name)
    if initial is None:
        raise RuntimeError("%s has no measured position before command" % name)
    initial_position = float(initial["position"])
    deadline = time.monotonic() + 8.0
    stable_since = None
    peak_velocity = 0.0
    last = None
    next_publish = 0.0
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError("Gazebo exited while moving " + name)
        now = time.monotonic()
        if now >= next_publish:
            publish(topic, "gz.msgs.Double", payload)
            next_publish = now + 0.1
        joints, _, _ = tel.snapshot()
        last = joints.get(name)
        if last:
            peak_velocity = max(peak_velocity, abs(last["velocity"]))
            if abs(last["position"] - target) < 0.04 and abs(last["velocity"]) < 0.08:
                stable_since = time.monotonic() if stable_since is None else stable_since
                if time.monotonic() - stable_since >= 0.3:
                    measured_delta = float(last["position"]) - initial_position
                    if abs(measured_delta) < 0.03:
                        raise RuntimeError(
                            "%s measured position change %.6f rad is below 0.03 rad: %r"
                            % (name, measured_delta, last))
                    return {
                        "joint": name, "initial_position_rad": initial_position,
                        "target_rad": target,
                        "final_position_rad": last["position"],
                        "measured_position_delta_rad": measured_delta,
                        "final_velocity_rad_s": last["velocity"],
                        "peak_abs_velocity_rad_s": peak_velocity,
                        "status": "passed",
                    }
            else:
                stable_since = None
        time.sleep(0.02)
    raise RuntimeError("%s target %.3f did not settle; last=%r" % (name, target, last))


def pose_excursion(samples, origin):
    max_xy = 0.0
    max_yaw = 0.0
    for _, pose in samples:
        max_xy = max(max_xy, math.hypot(pose[0] - origin[0], pose[1] - origin[1]))
        delta = math.atan2(math.sin(pose[3] - origin[3]), math.cos(pose[3] - origin[3]))
        max_yaw = max(max_yaw, abs(delta))
    return max_xy, max_yaw


def main():
    os.makedirs(LOG_DIR, exist_ok=True)
    report = {"result": "FAIL", "joint_stages": [], "base_drive_challenge": {}}
    process = None
    try:
        report.update(make_world())
        env = os.environ.copy()
        env.setdefault("DISPLAY", ":0")
        process = subprocess.Popen(
            ["gz", "sim", "-s", "-r", WORLD_OUT], cwd=ROOT, env=env,
            stdout=open(LOG_OUT, "w", encoding="utf-8"),
            stderr=subprocess.STDOUT, start_new_session=True,
        )
        tel = Telemetry()
        wait_ready(tel, process)
        report["initial_joint_positions"] = tel.snapshot()[0]
        report["initial_chassis_pose"] = tel.snapshot()[1]
        for name, target in ARM_JOINTS.items():
            report["joint_stages"].append(move_joint(tel, process, name, target))
            report["joint_stages"].append(move_joint(tel, process, name, 0.0))
        # A real DiffDrive command must not translate or rotate the world-fixed
        # chassis. This is an independent check from arm articulation.
        _, origin, samples_before = tel.snapshot()
        start_index = len(samples_before)
        publish(BASE_CMD_TOPIC, "gz.msgs.Twist",
                "linear: {x: 0.12} angular: {z: 0.15}")
        time.sleep(1.0)
        publish(BASE_CMD_TOPIC, "gz.msgs.Twist",
                "linear: {x: 0.0} angular: {z: 0.0}")
        time.sleep(0.25)
        _, end_pose, samples = tel.snapshot()
        challenge_samples = samples[start_index:]
        max_xy, max_yaw = pose_excursion(challenge_samples, origin)
        report["base_drive_challenge"] = {
            "command": {"linear_x_m_s": 0.12, "angular_z_rad_s": 0.15, "duration_s": 1.0},
            "start_pose": origin,
            "end_pose": end_pose,
            "max_displacement_mm": max_xy * 1000.0,
            "max_yaw_delta_deg": math.degrees(max_yaw),
            "pose_samples": len(challenge_samples),
            "status": "passed" if max_xy <= MAX_BASE_DISPLACEMENT_M and max_yaw <= MAX_BASE_YAW_RAD else "failed",
        }
        if report["base_drive_challenge"]["status"] != "passed":
            raise RuntimeError("world-fixed chassis moved under DiffDrive challenge")
        report["result"] = "PASS"
        return 0
    except Exception as exc:
        report["error"] = str(exc)
        return 1
    finally:
        if process is not None:
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
        with open(REPORT_OUT, "w", encoding="utf-8") as stream:
            json.dump(report, stream, indent=2, sort_keys=True)
            stream.write("\n")
        print(json.dumps(report, indent=2, sort_keys=True))


if __name__ == "__main__":
    raise SystemExit(main())
