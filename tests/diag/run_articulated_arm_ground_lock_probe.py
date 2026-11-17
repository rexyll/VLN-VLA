#!/usr/bin/env python3
"""Minimal Gazebo Sim 8.15 test: can an internal revolute joint move while
its robot chassis is attached to a static DetachableJoint anchor?
"""
import json
import math
import os
import subprocess
import threading
import time

import gz.transport13 as gz_transport
from gz.msgs10.model_pb2 import Model
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.stringmsg_pb2 import StringMsg

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
WORLD = os.path.join(ROOT, "tests", "diag", "articulated_arm_ground_lock_test.sdf")
STATE = "/world/arm_lock_probe/chassis_lock/state"
ATTACH = "/world/arm_lock_probe/chassis_lock/attach"
DETACH = "/world/arm_lock_probe/chassis_lock/detach"
JOINTS = "/world/arm_lock_probe/model/robot/joint_state"
POSES = "/world/arm_lock_probe/dynamic_pose/info"
CMD = "/arm_lock_probe/cmd/probe_arm_joint"
TARGET = 0.45

class Telemetry:
    def __init__(self):
        self.lock = threading.Lock()
        self.state = None
        self.joint = None
        self.pose = None
        self.node = gz_transport.Node()
        for typ, topic, cb in ((StringMsg, STATE, self.on_state),
                               (Model, JOINTS, self.on_joint),
                               (Pose_V, POSES, self.on_pose)):
            if not self.node.subscribe(typ, topic, cb):
                raise RuntimeError("subscribe failed: " + topic)

    def on_state(self, msg):
        value = str(msg.data).strip().lower()
        if value in ("attached", "detached"):
            with self.lock:
                self.state = value

    def on_joint(self, msg):
        for joint in msg.joint:
            if joint.name == "probe_arm_joint" and joint.HasField("axis1"):
                with self.lock:
                    self.joint = float(joint.axis1.position)

    def on_pose(self, msg):
        for item in msg.pose:
            if item.name == "robot":
                p = item.position
                q = item.orientation
                yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
                with self.lock:
                    self.pose = (float(p.x), float(p.y), float(p.z), yaw)

    def snapshot(self):
        with self.lock:
            return self.state, self.joint, self.pose

def publish(topic, msg_type, payload=""):
    cmd = ["gz", "topic", "-t", topic, "-m", msg_type]
    if payload:
        cmd += ["-p", payload]
    result = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            text=True, timeout=10)
    if result.returncode:
        raise RuntimeError("publish failed %s: %s" % (topic, result.stdout))

def wait_for(tel, predicate, label, timeout=8.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        snap = tel.snapshot()
        if predicate(snap):
            return snap
        time.sleep(0.02)
    raise RuntimeError("timeout waiting for %s; latest=%r" % (label, tel.snapshot()))

def move_and_measure(tel, target, label):
    publish(CMD, "gz.msgs.Double", "data: %.6f" % target)
    snap = wait_for(tel, lambda s: s[1] is not None and abs(s[1]-target) < 0.08,
                    label + " joint target", timeout=10.0)
    return snap

def run():
    if not os.path.isfile(WORLD):
        raise RuntimeError("missing world " + WORLD)
    os.makedirs(os.path.join(ROOT, "vla_ws", "logs"), exist_ok=True)
    log_path = os.path.join(ROOT, "vla_ws", "logs", "articulated_arm_ground_lock_probe.log")
    with open(log_path, "w", encoding="utf-8") as log:
        proc = subprocess.Popen(["gz", "sim", "-s", "-r", WORLD], cwd=ROOT,
                                stdout=log, stderr=subprocess.STDOUT,
                                start_new_session=True)
        tel = None
        report = {"world": os.path.relpath(WORLD, ROOT), "target_rad": TARGET, "stages": []}
        try:
            tel = Telemetry()
            wait_for(tel, lambda s: s[0] == "attached" and s[1] is not None and s[2] is not None,
                     "initial attached state and telemetry", timeout=30.0)
            initial = tel.snapshot()
            report["initial_pose"] = initial[2]

            publish(DETACH, "gz.msgs.Empty")
            wait_for(tel, lambda s: s[0] == "detached", "detach state")
            detached_start = tel.snapshot()[2]
            move_and_measure(tel, TARGET, "detached")
            detached_end = tel.snapshot()
            report["stages"].append({"name": "detached", "joint_rad": detached_end[1],
                                     "chassis_pose": detached_end[2]})

            move_and_measure(tel, 0.0, "detached reset")
            publish(ATTACH, "gz.msgs.Empty")
            wait_for(tel, lambda s: s[0] == "attached", "attach state")
            attached_start = tel.snapshot()[2]
            move_and_measure(tel, TARGET, "attached")
            attached_end = tel.snapshot()
            delta = math.sqrt(sum((attached_end[2][i]-attached_start[i])**2 for i in range(3)))
            yaw_delta = abs(attached_end[2][3]-attached_start[3])
            report["stages"].append({"name": "attached", "joint_rad": attached_end[1],
                                     "chassis_pose": attached_end[2],
                                     "chassis_displacement_mm": delta*1000.0,
                                     "chassis_yaw_delta_rad": yaw_delta})
            if abs(detached_end[1]) < 0.25:
                raise RuntimeError("detached revolute joint did not move enough")
            if abs(attached_end[1]) < 0.25:
                raise RuntimeError("attached revolute joint did not move enough")
            if delta > 0.002 or yaw_delta > math.radians(0.5):
                raise RuntimeError("attached chassis moved beyond physical hold limits")
            report["result"] = "PASS"
            print(json.dumps(report, indent=2))
        finally:
            try:
                os.killpg(proc.pid,  signal.SIGTERM)
                proc.wait(timeout=10)
            except Exception:
                try:
                    os.killpg(proc.pid, signal.SIGKILL)
                except Exception:
                    pass

if __name__ == "__main__":
    run()
