#!/usr/bin/env python3
"""Minimal Gazebo Sim 8.15 test: can an internal revolute joint move while
its robot chassis is attached to a static DetachableJoint anchor?
"""
import json
import math
import os
import signal
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
        self.joint = {}
        self.samples = []
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
            if joint.name in ("probe_arm_joint", "probe_arm_force_joint") and joint.HasField("axis1"):
                with self.lock:
                    sample = {"position": float(joint.axis1.position), "velocity": float(joint.axis1.velocity), "effort": float(joint.axis1.force), "time_monotonic": time.monotonic()}
                    self.joint[joint.name] = sample
                    self.samples.append({"joint": joint.name, **sample})
                    if len(self.samples) > 4000:
                        self.samples = self.samples[-4000:]

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
            return self.state, {k: dict(v) for k, v in self.joint.items()}, self.pose

    def sample_snapshot(self):
        with self.lock:
            return [dict(x) for x in self.samples]

def publish(topic, msg_type, payload=""):
    cmd = ["gz", "topic", "-t", topic, "-m", msg_type]
    if msg_type == "gz.msgs.Empty":
        cmd += ["-p", " "]
    elif payload:
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

def move_and_measure(tel, joint_name, target, label):
    topic = "/arm_lock_probe/cmd/" + joint_name
    start_index = len(tel.sample_snapshot())
    publish(topic, "gz.msgs.Double", "data: %.6f" % target)
    deadline = time.monotonic() + 10.0
    stable_since = None
    peak_position = None
    max_velocity = 0.0
    max_effort = 0.0
    last = None
    while time.monotonic() < deadline:
        snap = tel.snapshot()
        value = snap[1].get(joint_name)
        if value is not None:
            last = value
            peak_position = value["position"] if peak_position is None else (max(peak_position, value["position"]) if target >= 0 else min(peak_position, value["position"]))
            max_velocity = max(max_velocity, abs(value["velocity"]))
            max_effort = max(max_effort, abs(value["effort"]))
            if abs(value["position"] - target) < 0.08 and abs(value["velocity"]) < 0.05:
                stable_since = time.monotonic() if stable_since is None else stable_since
                if time.monotonic() - stable_since >= 0.5:
                    return {"snapshot": snap, "metrics": {"joint": joint_name, "target_rad": target, "final_position_rad": value["position"], "final_velocity_rad_s": value["velocity"], "final_effort_nm": value["effort"], "peak_position_rad": peak_position, "max_abs_velocity_rad_s": max_velocity, "max_abs_effort_nm": max_effort, "stable_duration_s": 0.5, "samples": tel.sample_snapshot()[start_index:]}}
            else:
                stable_since = None
        time.sleep(0.02)
    raise RuntimeError("%s did not settle; last=%r; samples=%r" % (label, last, tel.sample_snapshot()[start_index:][-30:]))

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
            wait_for(tel, lambda s: s[0] == "attached" and s[1].get("probe_arm_joint") is not None and s[1].get("probe_arm_force_joint") is not None and s[2] is not None,
                     "initial attached state and telemetry", timeout=30.0)
            initial = tel.snapshot()
            report["initial_pose"] = initial[2]

            publish(DETACH, "gz.msgs.Empty")
            wait_for(tel, lambda s: s[0] == "detached", "detach state")
            detached_start = tel.snapshot()[2]
            report["current_stage"] = "detached_velocity_mode"
            detached_velocity = move_and_measure(tel, "probe_arm_joint", TARGET, "detached velocity-mode")
            report["stages"].append({"name": "detached_velocity_mode", **detached_velocity["metrics"]})
            report["current_stage"] = "detached_velocity_reset"
            move_and_measure(tel, "probe_arm_joint", 0.0, "detached velocity-mode reset")
            report["current_stage"] = "detached_force_mode"
            detached_force = move_and_measure(tel, "probe_arm_force_joint", TARGET, "detached force-mode")
            report["stages"].append({"name": "detached_force_mode", **detached_force["metrics"]})
            report["current_stage"] = "detached_force_reset"
            move_and_measure(tel, "probe_arm_force_joint", 0.0, "detached force-mode reset")
            detached_end = tel.snapshot()
            detached_delta = math.sqrt(sum((detached_end[2][i]-detached_start[i])**2 for i in range(3)))
            report["stages"].append({"name": "detached",
                                     "velocity_mode_joint_rad": detached_velocity["snapshot"][1].get("probe_arm_joint", {}).get("position"),
                                     "force_mode_joint_rad": detached_force["snapshot"][1].get("probe_arm_force_joint", {}).get("position"),
                                     "chassis_start_pose": detached_start,
                                     "chassis_pose": detached_end[2],
                                     "chassis_displacement_mm": detached_delta*1000.0})

            publish(ATTACH, "gz.msgs.Empty")
            wait_for(tel, lambda s: s[0] == "attached", "attach state")
            attached_start = tel.snapshot()[2]
            report["attached_start_pose"] = attached_start
            report["current_stage"] = "attached_velocity_mode"
            attached_velocity = move_and_measure(tel, "probe_arm_joint", TARGET, "attached velocity-mode")
            report["stages"].append({"name": "attached_velocity_mode", **attached_velocity["metrics"]})
            report["current_stage"] = "attached_force_mode"
            attached_force = move_and_measure(tel, "probe_arm_force_joint", TARGET, "attached force-mode")
            report["stages"].append({"name": "attached_force_mode", **attached_force["metrics"]})
            attached_end = tel.snapshot()
            delta = math.sqrt(sum((attached_end[2][i]-attached_start[i])**2 for i in range(3)))
            yaw_delta = abs(attached_end[2][3]-attached_start[3])
            report["stages"].append({"name": "attached",
                                     "velocity_mode_joint_rad": attached_velocity["snapshot"][1].get("probe_arm_joint", {}).get("position"),
                                     "force_mode_joint_rad": attached_force["snapshot"][1].get("probe_arm_force_joint", {}).get("position"),
                                     "chassis_pose": attached_end[2],
                                     "chassis_displacement_mm": delta*1000.0,
                                     "chassis_yaw_delta_rad": yaw_delta})
            if abs(detached_velocity["snapshot"][1].get("probe_arm_joint", {}).get("position", 0.0)) < 0.25:
                raise RuntimeError("detached velocity-mode joint did not move enough")
            if abs(detached_force["snapshot"][1].get("probe_arm_force_joint", {}).get("position", 0.0)) < 0.25:
                raise RuntimeError("detached force-mode joint did not move enough")
            if abs(attached_velocity["snapshot"][1].get("probe_arm_joint", {}).get("position", 0.0)) < 0.25:
                raise RuntimeError("attached velocity-mode joint did not move enough")
            if abs(attached_force["snapshot"][1].get("probe_arm_force_joint", {}).get("position", 0.0)) < 0.25:
                raise RuntimeError("attached force-mode joint did not move enough")
            if delta > 0.002 or yaw_delta > math.radians(0.5):
                raise RuntimeError("attached chassis moved beyond physical hold limits")
            report["result"] = "PASS"
            print(json.dumps(report, indent=2))
        except Exception as exc:
            report["result"] = "FAIL"
            report["error"] = str(exc)
            report["last_telemetry"] = tel.snapshot() if tel is not None else None
            report["last_samples"] = tel.sample_snapshot()[-200:] if tel is not None else None
            print(json.dumps(report, indent=2))
            raise
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
