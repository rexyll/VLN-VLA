#!/usr/bin/env python3
"""Gazebo Sim 8 regression: production velocity-mode arm joint moves while
its chassis is detached and while held by a static DetachableJoint anchor.
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
WORLD = os.environ.get(
    "ARM_LOCK_PROBE_WORLD",
    os.path.join(ROOT, "tests", "diag", "articulated_arm_ground_lock_test.sdf"),
)
CONTROLLER_MODE = os.environ.get(
    "ARM_LOCK_PROBE_MODE", "use_velocity_commands=true"
)
LOG_NAME = os.environ.get(
    "ARM_LOCK_PROBE_LOG", "articulated_arm_ground_lock_probe.log"
)
STATE = "/world/arm_lock_probe/chassis_lock/state"
ATTACH = "/world/arm_lock_probe/chassis_lock/attach"
DETACH = "/world/arm_lock_probe/chassis_lock/detach"
JOINTS = "/world/arm_lock_probe/model/robot/joint_state"
POSES = "/world/arm_lock_probe/dynamic_pose/info"
JOINT = "probe_arm_joint"
TARGET = 0.45
MAX_BASE_DISPLACEMENT_M = 0.002
MAX_BASE_YAW_RAD = math.radians(0.5)


class Telemetry:
    def __init__(self):
        self.lock = threading.Lock()
        self.state = None
        self.joint = None
        self.samples = []
        self.pose = None
        self.pose_samples = []
        self.node = gz_transport.Node()
        for typ, topic, cb in (
            (StringMsg, STATE, self.on_state),
            (Model, JOINTS, self.on_joint),
            (Pose_V, POSES, self.on_pose),
        ):
            if not self.node.subscribe(typ, topic, cb):
                raise RuntimeError("subscribe failed: " + topic)

    def on_state(self, msg):
        value = str(msg.data).strip().lower()
        if value in ("attached", "detached"):
            with self.lock:
                self.state = value

    def on_joint(self, msg):
        for joint in msg.joint:
            if joint.name == JOINT and joint.HasField("axis1"):
                sample = {
                    "position": float(joint.axis1.position),
                    "velocity": float(joint.axis1.velocity),
                    "effort": float(joint.axis1.force),
                    "time_monotonic": time.monotonic(),
                }
                with self.lock:
                    self.joint = sample
                    self.samples.append(dict(sample))

    def on_pose(self, msg):
        for item in msg.pose:
            if item.name == "robot":
                p = item.position
                q = item.orientation
                yaw = math.atan2(
                    2 * (q.w * q.z + q.x * q.y),
                    1 - 2 * (q.y * q.y + q.z * q.z),
                )
                sample = (float(p.x), float(p.y), float(p.z), yaw)
                with self.lock:
                    self.pose = sample
                    self.pose_samples.append((time.monotonic(), sample))

    def snapshot(self):
        with self.lock:
            return self.state, dict(self.joint) if self.joint else None, self.pose

    def sample_snapshot(self):
        with self.lock:
            return [dict(x) for x in self.samples]

    def pose_snapshot(self):
        with self.lock:
            return list(self.pose_samples)


def publish(topic, msg_type, payload=""):
    cmd = ["gz", "topic", "-t", topic, "-m", msg_type]
    if msg_type == "gz.msgs.Empty":
        cmd += ["-p", " "]
    elif payload:
        cmd += ["-p", payload]
    result = subprocess.run(
        cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, timeout=10,
    )
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
    start_index = len(tel.sample_snapshot())
    publish("/arm_lock_probe/cmd/" + JOINT, "gz.msgs.Double",
            "data: %.6f" % target)
    deadline = time.monotonic() + 10.0
    stable_since = None
    peak_position = None
    max_velocity = 0.0
    last = None
    while time.monotonic() < deadline:
        snap = tel.snapshot()
        value = snap[1]
        if value is not None:
            last = value
            peak_position = (
                value["position"] if peak_position is None
                else (max(peak_position, value["position"])
                      if target >= 0 else min(peak_position, value["position"]))
            )
            max_velocity = max(max_velocity, abs(value["velocity"]))
            if (abs(value["position"] - target) < 0.08
                    and abs(value["velocity"]) < 0.05):
                stable_since = time.monotonic() if stable_since is None else stable_since
                if time.monotonic() - stable_since >= 0.5:
                    metrics = {
                        "joint": JOINT,
                        "mode": "use_velocity_commands=true",
                        "target_rad": target,
                        "final_position_rad": value["position"],
                        "final_velocity_rad_s": value["velocity"],
                        "final_effort_nm": value["effort"],
                        "peak_position_rad": peak_position,
                        "max_abs_velocity_rad_s": max_velocity,
                        "stable_duration_s": 0.5,
                        "samples": tel.sample_snapshot()[start_index:],
                    }
                    if max_velocity < 0.05:
                        raise RuntimeError("%s reported no measurable joint motion" % label)
                    return {"snapshot": snap, "metrics": metrics}
            else:
                stable_since = None
        time.sleep(0.02)
    raise RuntimeError(
        "%s did not settle; last=%r; samples=%r"
        % (label, last, tel.sample_snapshot()[start_index:][-30:])
    )


def base_excursion(samples, origin):
    max_displacement = 0.0
    max_yaw = 0.0
    for _, pose in samples:
        displacement = math.sqrt(sum((pose[i] - origin[i]) ** 2 for i in range(3)))
        yaw_delta = abs(math.atan2(
            math.sin(pose[3] - origin[3]), math.cos(pose[3] - origin[3])
        ))
        max_displacement = max(max_displacement, displacement)
        max_yaw = max(max_yaw, yaw_delta)
    return max_displacement, max_yaw


def run():
    if not os.path.isfile(WORLD):
        raise RuntimeError("missing world " + WORLD)
    os.makedirs(os.path.join(ROOT, "vla_ws", "logs"), exist_ok=True)
    log_path = os.path.join(ROOT, "vla_ws", "logs", LOG_NAME)
    with open(log_path, "w", encoding="utf-8") as log:
        proc = subprocess.Popen(
            ["gz", "sim", "-s", "-r", WORLD], cwd=ROOT,
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
        )
        tel = None
        report = {
            "world": os.path.relpath(WORLD, ROOT),
            "controller_mode": CONTROLLER_MODE,
            "target_rad": TARGET,
            "stages": [],
        }
        try:
            tel = Telemetry()
            wait_for(
                tel,
                lambda s: s[0] == "attached" and s[1] is not None and s[2] is not None,
                "initial attached state and telemetry",
                timeout=30.0,
            )
            report["initial_pose"] = tel.snapshot()[2]

            publish(DETACH, "gz.msgs.Empty")
            wait_for(tel, lambda s: s[0] == "detached", "detach state")
            detached_start = tel.snapshot()[2]
            detached_start_index = len(tel.pose_snapshot())
            report["current_stage"] = "detached_positive_velocity_mode"
            detached_positive = move_and_measure(tel, TARGET, "detached positive velocity-mode")
            report["stages"].append({
                "name": "detached_positive_velocity_mode",
                **detached_positive["metrics"],
            })
            report["current_stage"] = "detached_reset"
            move_and_measure(tel, 0.0, "detached reset")
            report["current_stage"] = "detached_negative_velocity_mode"
            detached_negative = move_and_measure(tel, -TARGET, "detached negative velocity-mode")
            report["stages"].append({
                "name": "detached_negative_velocity_mode",
                **detached_negative["metrics"],
            })
            report["current_stage"] = "detached_negative_reset"
            move_and_measure(tel, 0.0, "detached negative reset")
            detached_end = tel.snapshot()[2]
            detached_delta = math.sqrt(sum(
                (detached_end[i] - detached_start[i]) ** 2 for i in range(3)
            ))
            report["stages"].append({
                "name": "detached_chassis",
                "chassis_start_pose": detached_start,
                "chassis_end_pose": detached_end,
                "chassis_displacement_mm": detached_delta * 1000.0,
                "pose_samples": len(tel.pose_snapshot()) - detached_start_index,
            })

            publish(ATTACH, "gz.msgs.Empty")
            wait_for(tel, lambda s: s[0] == "attached", "attach state")
            attached_start = tel.snapshot()[2]
            attached_start_index = len(tel.pose_snapshot())
            report["attached_start_pose"] = attached_start
            report["current_stage"] = "attached_positive_velocity_mode"
            attached_positive = move_and_measure(tel, TARGET, "attached positive velocity-mode")
            report["stages"].append({
                "name": "attached_positive_velocity_mode",
                **attached_positive["metrics"],
            })
            report["current_stage"] = "attached_negative_velocity_mode"
            attached_negative = move_and_measure(tel, -TARGET, "attached negative velocity-mode")
            report["stages"].append({
                "name": "attached_negative_velocity_mode",
                **attached_negative["metrics"],
            })
            attached_end = tel.snapshot()[2]
            attached_samples = tel.pose_snapshot()[attached_start_index:]
            max_displacement, max_yaw = base_excursion(attached_samples, attached_start)
            report["stages"].append({
                "name": "attached_chassis",
                "chassis_start_pose": attached_start,
                "chassis_end_pose": attached_end,
                "max_chassis_displacement_mm": max_displacement * 1000.0,
                "max_chassis_yaw_delta_deg": math.degrees(max_yaw),
                "pose_samples": len(attached_samples),
            })

            if abs(detached_positive["snapshot"][1]["position"]) < 0.25:
                raise RuntimeError("detached positive velocity-mode joint did not move enough")
            if abs(detached_negative["snapshot"][1]["position"]) < 0.25:
                raise RuntimeError("detached negative velocity-mode joint did not move enough")
            if abs(attached_positive["snapshot"][1]["position"]) < 0.25:
                raise RuntimeError("attached positive velocity-mode joint did not move enough")
            if abs(attached_negative["snapshot"][1]["position"]) < 0.25:
                raise RuntimeError("attached negative velocity-mode joint did not move enough")
            if max_displacement > MAX_BASE_DISPLACEMENT_M or max_yaw > MAX_BASE_YAW_RAD:
                raise RuntimeError(
                    "attached chassis exceeded hold limits: displacement=%.3fmm yaw=%.4fdeg"
                    % (max_displacement * 1000.0, math.degrees(max_yaw))
                )
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
                os.killpg(proc.pid, signal.SIGTERM)
                proc.wait(timeout=10)
            except Exception:
                try:
                    os.killpg(proc.pid, signal.SIGKILL)
                except Exception:
                    pass


if __name__ == "__main__":
    run()
