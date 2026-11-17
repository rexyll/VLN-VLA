#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Probe a same-model, world-rooted detachable chassis lock.

The robot model gets a tiny world-fixed anchor link. Gazebo's DetachableJoint
system runs inside that model and uses child_model=__model__ to connect the
anchor link to robot::chassis. This is diagnostic-only; production SDF is not
modified.
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
from gz.msgs10.stringmsg_pb2 import StringMsg

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
SOURCE = os.path.join(ROOT, "vla_ws/src/vla_follow/worlds/work_area.sdf")
LOG_DIR = os.path.join(ROOT, "vla_ws/logs")
WORLD_OUT = os.path.join(LOG_DIR, "full_robot_internal_detachable.sdf")
LOG_OUT = os.path.join(LOG_DIR, "full_robot_internal_detachable_gz.log")
REPORT_OUT = os.path.join(LOG_DIR, "full_robot_internal_detachable.json")
STATE_TOPIC = "/world/follow_world/chassis_lock/state"
ATTACH_TOPIC = "/world/follow_world/chassis_lock/attach"
DETACH_TOPIC = "/world/follow_world/chassis_lock/detach"
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
    with open(SOURCE, encoding="utf-8") as f:
        source = f.read()
    source, n = re.subn(
        r"\s*<!-- Physical chassis immobilizer[\s\S]*?</model>\s*",
        "\n", source, count=1
    )
    if n != 1:
        raise RuntimeError("could not remove exactly one external anchor model")
    source, n = re.subn(
        r"\s*<plugin filename=\"libGraspCarry\.so\" name=\"vla::GraspCarry\">"
        r"[\s\S]*?</plugin>\s*", "\n", source, count=1
    )
    if n != 1:
        raise RuntimeError("could not remove exactly one GraspCarry world plugin")
    start = re.search(r'<model name="robot" canonical_link="chassis">', source)
    if not start:
        raise RuntimeError("production robot model not found")
    source = source[:start.start()] + source[start.start():].replace(
        '<model name="robot" canonical_link="chassis">',
        '<model name="robot" canonical_link="world_anchor">', 1
    )
    start = re.search(r'<model name="robot" canonical_link="world_anchor">', source)
    end = source.find("</model>", start.start())
    if end < 0:
        raise RuntimeError("robot model closing tag not found")
    insert = """
      <!-- Diagnostic-only world root for same-model detachable lock. -->
      <link name="world_anchor">
        <pose>0 0 0.13 0 0 0</pose>
        <inertial>
          <mass>0.01</mass>
          <inertia><ixx>0.000001</ixx><iyy>0.000001</iyy><izz>0.000001</izz>
            <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
        </inertial>
      </link>
      <joint name="world_anchor_fixed" type="fixed">
        <parent>world</parent><child>world_anchor</child>
      </joint>
      <plugin filename="gz-sim-detachable-joint-system"
              name="gz::sim::systems::DetachableJoint">
        <parent_link>world_anchor</parent_link>
        <child_model>__model__</child_model>
        <child_link>chassis</child_link>
        <attach_topic>/world/follow_world/chassis_lock/attach</attach_topic>
        <detach_topic>/world/follow_world/chassis_lock/detach</detach_topic>
        <output_topic>/world/follow_world/chassis_lock/state</output_topic>
      </plugin>
"""
    source = source[:end] + insert + source[end:]
    os.makedirs(LOG_DIR, exist_ok=True)
    with open(WORLD_OUT, "w", encoding="utf-8") as f:
        f.write(source)
    check = subprocess.run(
        ["gz", "sdf", "-k", WORLD_OUT], cwd=ROOT, text=True,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=30
    )
    if check.returncode:
        raise RuntimeError("generated SDF invalid:\n" + check.stdout)
    return check.stdout[-3000:]


class Telemetry:
    def __init__(self):
        self.lock = threading.Lock()
        self.joints = {}
        self.pose = None
        self.samples = []
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
        for j in msg.joint:
            if j.name in ARM_JOINTS and j.HasField("axis1"):
                values[j.name] = (float(j.axis1.position), float(j.axis1.velocity))
        with self.lock:
            self.joints.update(values)

    def on_pose(self, msg):
        for item in msg.pose:
            if item.name == "robot":
                p, q = item.position, item.orientation
                yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
                pose = (float(p.x), float(p.y), float(p.z), yaw)
                with self.lock:
                    self.pose = pose
                    self.samples.append((time.monotonic(), pose))
                return

    def on_state(self, msg):
        state = str(msg.data).strip().lower()
        if state in ("attached", "detached"):
            with self.lock:
                self.state = state
                self.states.append(state)

    def snapshot(self):
        with self.lock:
            return dict(self.joints), self.pose, list(self.samples), self.state, list(self.states)


def ensure_alive(proc, where):
    if proc.poll() is not None:
        raise RuntimeError("Gazebo exited during %s (rc=%s)" % (where, proc.returncode))


def publish(topic, typ, payload):
    r = subprocess.run(
        ["gz", "topic", "-t", topic, "-m", typ, "-p", payload],
        cwd=ROOT, text=True, stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT, timeout=10
    )
    if r.returncode:
        raise RuntimeError("publish failed %s: %s" % (topic, r.stdout))


def wait_for(tel, proc, predicate, timeout, label):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        ensure_alive(proc, label)
        if predicate(tel.snapshot()):
            return
        time.sleep(0.02)
    raise RuntimeError("timeout waiting for " + label + "; state=" + repr(tel.snapshot()[3]))


def request_state(tel, proc, target):
    topic = ATTACH_TOPIC if target == "attached" else DETACH_TOPIC
    deadline = time.monotonic() + 6.0
    attempts = 0
    while time.monotonic() < deadline:
        ensure_alive(proc, "request " + target)
        if tel.snapshot()[3] == target:
            return attempts
        publish(topic, "gz.msgs.Empty", " ")
        attempts += 1
        end = min(deadline, time.monotonic() + 0.25)
        while time.monotonic() < end:
            if tel.snapshot()[3] == target:
                return attempts
            time.sleep(0.02)
    raise RuntimeError("state transition to %s timed out after %d requests" % (target, attempts))


def move_joint(tel, proc, name, target):
    publish("/drink/cmd/" + name, "gz.msgs.Double", "data: %.6f" % target)
    deadline = time.monotonic() + 8.0
    stable = None
    peak = 0.0
    last = None
    while time.monotonic() < deadline:
        ensure_alive(proc, "move " + name)
        last = tel.snapshot()[0].get(name)
        if last:
            peak = max(peak, abs(last[1]))
            if abs(last[0] - target) < 0.04 and abs(last[1]) < 0.08:
                stable = time.monotonic() if stable is None else stable
                if time.monotonic() - stable >= 0.3:
                    if peak < 0.03:
                        raise RuntimeError("%s did not move: %r" % (name, last))
                    return {"joint": name, "target_rad": target,
                            "final_position_rad": last[0],
                            "final_velocity_rad_s": last[1],
                            "peak_abs_velocity_rad_s": peak, "status": "passed"}
            else:
                stable = None
        time.sleep(0.02)
    raise RuntimeError("%s target %.3f did not settle; last=%r" % (name, target, last))


def pose_excursion(samples, origin):
    xy = yaw = 0.0
    for _, p in samples:
        xy = max(xy, math.hypot(p[0]-origin[0], p[1]-origin[1]))
        dyaw = math.atan2(math.sin(p[3]-origin[3]), math.cos(p[3]-origin[3]))
        yaw = max(yaw, abs(dyaw))
    return xy, yaw


def drive_challenge(tel, proc, should_hold):
    _, origin, samples, _, _ = tel.snapshot()
    index = len(samples)
    publish(BASE_CMD_TOPIC, "gz.msgs.Twist", "linear: {x: 0.12} angular: {z: 0.15}")
    time.sleep(1.0)
    publish(BASE_CMD_TOPIC, "gz.msgs.Twist", "linear: {x: 0.0} angular: {z: 0.0}")
    time.sleep(0.3)
    _, end, all_samples, _, _ = tel.snapshot()
    xy, yaw = pose_excursion(all_samples[index:], origin)
    moved = xy >= 0.01
    if should_hold and (xy > MAX_BASE_DISPLACEMENT_M or yaw > MAX_BASE_YAW_RAD):
        raise RuntimeError("attached chassis moved %.3fmm / %.4fdeg" % (xy*1000, math.degrees(yaw)))
    if not should_hold and not moved:
        raise RuntimeError("detached chassis failed to move >=10mm")
    return {"status": "passed", "max_displacement_mm": xy*1000,
            "max_yaw_delta_deg": math.degrees(yaw),
            "start_pose": origin, "end_pose": end,
            "pose_samples": len(all_samples[index:])}


def main():
    report = {"result": "FAIL", "joint_stages": [], "state_transitions": [], "drive_challenges": []}
    proc = None
    log = None
    try:
        report["sdf_check"] = make_world()
        log = open(LOG_OUT, "w", encoding="utf-8")
        proc = subprocess.Popen(["gz", "sim", "-s", "-r", WORLD_OUT],
                                cwd=ROOT, env=os.environ.copy(),
                                stdout=log, stderr=subprocess.STDOUT,
                                start_new_session=True)
        tel = Telemetry()
        wait_for(tel, proc, lambda s: len(s[0]) == len(ARM_JOINTS) and s[1] is not None and s[3] is not None,
                 35, "joint/pose/initial lock telemetry")
        report["initial_state"] = tel.snapshot()[3]
        if tel.snapshot()[3] != "attached":
            raise RuntimeError("internal DetachableJoint did not initialize attached")
        for name, target in ARM_JOINTS.items():
            report["joint_stages"].append(move_joint(tel, proc, name, target))
            report["joint_stages"].append(move_joint(tel, proc, name, 0.0))
        report["drive_challenges"].append({"phase": "attached", **drive_challenge(tel, proc, True)})
        n = request_state(tel, proc, "detached")
        report["state_transitions"].append({"target": "detached", "attempts": n})
        report["drive_challenges"].append({"phase": "detached", **drive_challenge(tel, proc, False)})
        n = request_state(tel, proc, "attached")
        report["state_transitions"].append({"target": "attached", "attempts": n})
        for name, target in ARM_JOINTS.items():
            report["joint_stages"].append(move_joint(tel, proc, name, target))
            report["joint_stages"].append(move_joint(tel, proc, name, 0.0))
        report["drive_challenges"].append({"phase": "reattached", **drive_challenge(tel, proc, True)})
        n = request_state(tel, proc, "detached")
        report["state_transitions"].append({"target": "detached", "attempts": n})
        report["drive_challenges"].append({"phase": "final_detached", **drive_challenge(tel, proc, False)})
        report["state_messages"] = tel.snapshot()[4]
        report["result"] = "PASS"
        return 0
    except Exception as exc:
        report["error"] = str(exc)
        if proc is not None:
            try:
                report["state_messages"] = tel.snapshot()[4]
            except Exception:
                pass
        return 1
    finally:
        if proc is not None:
            try:
                publish(BASE_CMD_TOPIC, "gz.msgs.Twist", "linear: {x: 0.0} angular: {z: 0.0}")
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
        if log:
            log.close()
        os.makedirs(LOG_DIR, exist_ok=True)
        with open(REPORT_OUT, "w", encoding="utf-8") as f:
            json.dump(report, f, indent=2, sort_keys=True)
            f.write("\n")
        print(json.dumps(report, indent=2, sort_keys=True))


if __name__ == "__main__":
    raise SystemExit(main())
