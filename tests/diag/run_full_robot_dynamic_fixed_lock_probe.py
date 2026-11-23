#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Full robot runtime world-fixed joint lifecycle probe (diagnostic only).

The source work_area.sdf is copied into vla_ws/logs. The external
DetachableJoint anchor is removed only from that copy, and an experimental
DynamicChassisLock system plugin is inserted into the robot model. The plugin
creates/removes a world->chassis fixed joint through Gazebo Sim SdfEntityCreator.

Acceptance is physical, not entity-state-only:
* attached: six main arm joints move out and return; DiffDrive challenge <=2mm
  and <=0.5deg;
* detached: DiffDrive challenge moves the base >=10mm;
* at least three attach/detach cycles complete.
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
from google.protobuf import text_format
from gz.msgs10.double_pb2 import Double
from gz.msgs10.empty_pb2 import Empty
from gz.msgs10.model_pb2 import Model
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.stringmsg_pb2 import StringMsg
from gz.msgs10.twist_pb2 import Twist

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
SOURCE = os.path.join(ROOT, "vla_ws/src/vla_follow/worlds/work_area.sdf")
LOG_DIR = os.path.join(ROOT, "vla_ws/logs")
WORLD_OUT = os.path.join(LOG_DIR, "full_robot_dynamic_fixed_lock.sdf")
LOG_OUT = os.path.join(LOG_DIR, "full_robot_dynamic_fixed_lock_gz.log")
REPORT_OUT = os.path.join(LOG_DIR, "full_robot_dynamic_fixed_lock.json")
BUILD_DIR = os.path.join(ROOT, "vla_ws/src/vla_follow/gz_plugins/build")
ATTACH_TOPIC = "/world/follow_world/dynamic_chassis_lock/attach"
DETACH_TOPIC = "/world/follow_world/dynamic_chassis_lock/detach"
STATE_TOPIC = "/world/follow_world/dynamic_chassis_lock/state"
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
MIN_UNLOCKED_DISPLACEMENT_M = 0.01


def make_world():
    with open(SOURCE, encoding="utf-8") as stream:
        source = stream.read()
    source, removed = re.subn(
        r"\s*<!-- Physical chassis immobilizer[\s\S]*?</model>\s*",
        "\n", source, count=1
    )
    if removed != 1:
        raise RuntimeError("expected exactly one production external anchor block")
    robot = re.search(r'<model name="robot" canonical_link="chassis">', source)
    if not robot:
        raise RuntimeError("production robot model not found")
    robot_end = source.find("</model>", robot.start())
    if robot_end < 0:
        raise RuntimeError("robot model closing tag not found")
    plugin = """
      <!-- Diagnostic-only runtime fixed-joint lock; never loaded in production. -->
      <plugin filename="libDynamicChassisLock.so" name="vla::DynamicChassisLock">
        <attach_topic>/world/follow_world/dynamic_chassis_lock/attach</attach_topic>
        <detach_topic>/world/follow_world/dynamic_chassis_lock/detach</detach_topic>
        <state_topic>/world/follow_world/dynamic_chassis_lock/state</state_topic>
        <initially_locked>true</initially_locked>
      </plugin>
"""
    source = source[:robot_end] + plugin + source[robot_end:]
    os.makedirs(LOG_DIR, exist_ok=True)
    with open(WORLD_OUT, "w", encoding="utf-8") as stream:
        stream.write(source)
    check = subprocess.run(
        ["gz", "sdf", "-k", WORLD_OUT], cwd=ROOT, text=True,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=30
    )
    if check.returncode:
        raise RuntimeError("generated diagnostic SDF invalid:\\n" + check.stdout)
    return {
        "source_sdf": os.path.relpath(SOURCE, ROOT),
        "diagnostic_sdf": os.path.relpath(WORLD_OUT, ROOT),
        "production_source_modified": False,
        "external_detachable_anchor_removed_from_copy": True,
        "dynamic_fixed_lock_plugin_inserted": True,
        "sdf_check": check.stdout[-2000:],
    }


class Telemetry:
    def __init__(self):
        self.lock = threading.Lock()
        self.joints = {}
        self.pose = None
        self.pose_samples = []
        self.state = None
        self.states = []
        self.node = gz_transport.Node()
        for typ, topic, cb in (
            (Model, JOINT_TOPIC, self.on_joints),
            (Pose_V, POSE_TOPIC, self.on_pose),
            (StringMsg, STATE_TOPIC, self.on_state),
        ):
            if not self.node.subscribe(typ, topic, cb):
                raise RuntimeError("cannot subscribe " + topic)

    def on_joints(self, msg):
        values = {}
        for joint in msg.joint:
            if joint.name in ARM_JOINTS and joint.HasField("axis1"):
                values[joint.name] = {
                    "position": float(joint.axis1.position),
                    "velocity": float(joint.axis1.velocity),
                }
        with self.lock:
            self.joints.update(values)

    def on_pose(self, msg):
        for item in msg.pose:
            if item.name == "robot":
                p, q = item.position, item.orientation
                yaw = math.atan2(
                    2.0 * (q.w*q.z + q.x*q.y),
                    1.0 - 2.0 * (q.y*q.y + q.z*q.z),
                )
                pose = (float(p.x), float(p.y), float(p.z), yaw)
                with self.lock:
                    self.pose = pose
                    self.pose_samples.append((time.monotonic(), pose))
                return

    def on_state(self, msg):
        state = str(msg.data).strip().lower()
        with self.lock:
            self.state = state
            self.states.append(state)

    def snapshot(self):
        with self.lock:
            return (dict(self.joints), self.pose, list(self.pose_samples),
                    self.state, list(self.states))


class Publishers:
    TYPES = {"empty": Empty, "double": Double, "twist": Twist}

    def __init__(self):
        self.node = gz_transport.Node()
        self.publishers = {}

    def publish(self, topic, kind, payload=""):
        typ = self.TYPES[kind]
        pub = self.publishers.get(topic)
        if pub is None:
            pub = self.node.advertise(topic, typ)
            if not pub:
                raise RuntimeError("cannot advertise " + topic)
            self.publishers[topic] = pub
        msg = typ()
        if payload.strip():
            text_format.Parse(payload, msg)
        if not pub.publish(msg):
            raise RuntimeError("publish failed " + topic)


def ensure_alive(proc, where):
    if proc.poll() is not None:
        raise RuntimeError("Gazebo exited during %s (rc=%s)" % (where, proc.returncode))


def wait_until(tel, proc, predicate, timeout, label):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        ensure_alive(proc, label)
        snap = tel.snapshot()
        if predicate(snap):
            return snap
        if snap[3] and snap[3].startswith("fault"):
            raise RuntimeError(label + ": plugin reported " + snap[3])
        time.sleep(0.02)
    raise RuntimeError(label + " timeout; latest=" + repr(tel.snapshot()[3]))


def request_lock(tel, proc, pubs, locked):
    topic = ATTACH_TOPIC if locked else DETACH_TOPIC
    expected = "attached" if locked else "detached"
    for _ in range(8):
        if tel.snapshot()[3] == expected:
            return
        ensure_alive(proc, "request " + expected)
        pubs.publish(topic, "empty")
        try:
            wait_until(tel, proc, lambda s: s[3] == expected, 1.0, expected)
            return
        except RuntimeError as exc:
            if "plugin reported" in str(exc):
                raise
    raise RuntimeError("failed to transition dynamic lock to " + expected)


def move_joint(tel, proc, pubs, name, target):
    initial = tel.snapshot()[0].get(name)
    if initial is None:
        raise RuntimeError("missing initial measured state for " + name)
    initial_position = initial["position"]
    topic = "/drink/cmd/" + name
    payload = "data: %.6f" % target
    deadline = time.monotonic() + 8.0
    stable_since = None
    next_publish = 0.0
    last = None
    peak_velocity = 0.0
    while time.monotonic() < deadline:
        ensure_alive(proc, "move " + name)
        now = time.monotonic()
        if now >= next_publish:
            pubs.publish(topic, "double", payload)
            next_publish = now + 0.1
        last = tel.snapshot()[0].get(name)
        if last:
            peak_velocity = max(peak_velocity, abs(last["velocity"]))
            if abs(last["position"] - target) < 0.04 and abs(last["velocity"]) < 0.08:
                stable_since = now if stable_since is None else stable_since
                if now - stable_since >= 0.3:
                    delta = last["position"] - initial_position
                    if abs(delta) < 0.03:
                        raise RuntimeError("%s did not physically move: delta=%.6f" % (name, delta))
                    return {
                        "joint": name, "target_rad": target,
                        "initial_position_rad": initial_position,
                        "final_position_rad": last["position"],
                        "measured_position_delta_rad": delta,
                        "peak_abs_velocity_rad_s": peak_velocity,
                    }
            else:
                stable_since = None
        time.sleep(0.02)
    raise RuntimeError("%s did not settle at %.3f; last=%r" % (name, target, last))


def pose_excursion(samples, origin):
    max_xy = max_yaw = 0.0
    for _, pose in samples:
        max_xy = max(max_xy, math.hypot(pose[0]-origin[0], pose[1]-origin[1]))
        dyaw = math.atan2(math.sin(pose[3]-origin[3]), math.cos(pose[3]-origin[3]))
        max_yaw = max(max_yaw, abs(dyaw))
    return max_xy, max_yaw


def drive_challenge(tel, proc, pubs, should_hold, label):
    _, origin, samples, _, _ = tel.snapshot()
    if origin is None:
        raise RuntimeError(label + ": missing initial base pose")
    index = len(samples)
    pubs.publish(BASE_CMD_TOPIC, "twist", "linear: {x: 0.12} angular: {z: 0.15}")
    time.sleep(1.0)
    pubs.publish(BASE_CMD_TOPIC, "twist", "linear: {x: 0.0} angular: {z: 0.0}")
    time.sleep(0.75)
    ensure_alive(proc, label)
    _, end, all_samples, _, _ = tel.snapshot()
    window = all_samples[index:]
    if len(window) < 10:
        raise RuntimeError(label + ": insufficient chassis pose samples")
    xy, yaw = pose_excursion(window, origin)
    result = {
        "phase": label, "should_hold": should_hold,
        "max_displacement_mm": xy*1000.0,
        "max_yaw_delta_deg": math.degrees(yaw),
        "pose_samples": len(window), "start_pose": origin, "end_pose": end,
    }
    if should_hold:
        if xy > MAX_BASE_DISPLACEMENT_M or yaw > MAX_BASE_YAW_RAD:
            raise RuntimeError("%s lock challenge failed: %.3fmm / %.4fdeg" %
                               (label, xy*1000.0, math.degrees(yaw)))
    elif xy < MIN_UNLOCKED_DISPLACEMENT_M:
        raise RuntimeError("%s unlock challenge failed: only %.3fmm movement" %
                           (label, xy*1000.0))
    return result


def main():
    report = {"result": "FAIL", "cycles": [], "joint_stages": []}
    proc = None
    log_stream = None
    try:
        if not os.path.isfile(os.path.join(BUILD_DIR, "libDynamicChassisLock.so")):
            raise RuntimeError("libDynamicChassisLock.so is missing; build plugins first")
        report.update(make_world())
        env = os.environ.copy()
        env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = BUILD_DIR + os.pathsep + env.get("GZ_SIM_SYSTEM_PLUGIN_PATH", "")
        env.setdefault("DISPLAY", ":0")
        log_stream = open(LOG_OUT, "w", encoding="utf-8")
        proc = subprocess.Popen(
            ["gz", "sim", "-s", "-r", WORLD_OUT], cwd=ROOT, env=env,
            stdout=log_stream, stderr=subprocess.STDOUT, start_new_session=True,
        )
        tel = Telemetry()
        pubs = Publishers()
        wait_until(tel, proc,
                   lambda s: len(s[0]) == len(ARM_JOINTS) and s[1] is not None and s[3] == "attached",
                   60.0, "initial robot telemetry and startup lock")
        report["initial_pose"] = tel.snapshot()[1]

        for cycle in range(1, 4):
            cycle_result = {"cycle": cycle, "arm_motion": []}
            for name, target in ARM_JOINTS.items():
                cycle_result["arm_motion"].append(move_joint(tel, proc, pubs, name, target))
                cycle_result["arm_motion"].append(move_joint(tel, proc, pubs, name, 0.0))
            cycle_result["attached_drive_challenge"] = drive_challenge(
                tel, proc, pubs, True, "cycle_%d_attached" % cycle)
            request_lock(tel, proc, pubs, False)
            cycle_result["detached_drive_challenge"] = drive_challenge(
                tel, proc, pubs, False, "cycle_%d_detached" % cycle)
            request_lock(tel, proc, pubs, True)
            report["cycles"].append(cycle_result)

        request_lock(tel, proc, pubs, False)
        report["final_detached_drive_challenge"] = drive_challenge(
            tel, proc, pubs, False, "final_detached")
        report["state_messages"] = tel.snapshot()[4]
        report["result"] = "PASS"
        return 0
    except Exception as exc:
        report["error"] = str(exc)
        if proc is not None:
            try:
                with open(LOG_OUT, encoding="utf-8", errors="replace") as stream:
                    report["gazebo_log_tail"] = stream.read()[-6000:]
            except OSError:
                pass
        return 1
    finally:
        if proc is not None:
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
        if log_stream is not None:
            log_stream.close()
        os.makedirs(LOG_DIR, exist_ok=True)
        with open(REPORT_OUT, "w", encoding="utf-8") as stream:
            json.dump(report, stream, indent=2, sort_keys=True)
            stream.write("\n")
        print(json.dumps(report, indent=2, sort_keys=True))


if __name__ == "__main__":
    raise SystemExit(main())
