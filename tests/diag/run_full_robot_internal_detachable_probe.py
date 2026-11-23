#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Probe a same-model, world-rooted detachable chassis lock.

A world-fixed joint anchors robot::world_anchor to the world. The
DetachableJoint system is loaded inside the robot model and connects
world_anchor to robot::chassis using child_model=__model__. This avoids the
external cross-model anchor topology that froze the production robot's
articulated joints. The production GraspCarry world plugin remains enabled.
Only a disposable diagnostic SDF is modified; production SDF is untouched.
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
PLUGIN_SO = os.path.join(ROOT, "vla_ws/src/vla_follow/gz_plugins/build/libGraspCarry.so")
PLUGIN_READY_MARKER = "[GraspCarry] 就绪"


def make_world():
    with open(SOURCE, encoding="utf-8") as f:
        source = f.read()
    source, n = re.subn(
        r"\s*<!-- Physical chassis immobilizer[\s\S]*?</model>\s*",
        "\n", source, count=1
    )
    if n != 1:
        raise RuntimeError("could not remove exactly one external anchor model")
    # Keep the production pose-executor plugin enabled. Removing it would
    # confound the comparison: the candidate anchor topology must coexist with
    # the actual cup/kettle carry system used by the full DrinkTask.
    if not re.search(
            r'<plugin filename="libGraspCarry\.so" name="vla::GraspCarry">',
            source):
        raise RuntimeError("production GraspCarry world plugin is missing")
    start = re.search(r'<model name="robot" canonical_link="chassis">', source)
    if not start:
        raise RuntimeError("production robot model not found")
    # Keep chassis as canonical_link so dynamic_pose reports the mobile base
    # after the internal detachable joint is released.
    end = source.find("</model>", start.start())
    if end < 0:
        raise RuntimeError("robot model closing tag not found")
    internal_anchor = """
      <!-- Same-model tree topology: world -> world_anchor -> detachable chassis. -->
      <link name="world_anchor">
        <pose>0 0 0.13 0 0 0</pose>
      </link>
      <joint name="world_anchor_fixed_joint" type="fixed">
        <parent>world</parent>
        <child>world_anchor</child>
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
    source = source[:end] + internal_anchor + source[end:]
    if "ground_lock_anchor" in source:
        raise RuntimeError("generated world still contains the external anchor model")
    if source.count('<link name="world_anchor">') != 1:
        raise RuntimeError("same-model world_anchor link insertion is not unique")
    if source.count('<joint name="world_anchor_fixed_joint"') != 1:
        raise RuntimeError("world_anchor fixed joint insertion is not unique")
    if source.count('name="gz::sim::systems::DetachableJoint"') != 1:
        raise RuntimeError("same-model DetachableJoint insertion is not unique")
    expected_topology = (
        "<parent>world</parent>\n"
        "        <child>world_anchor</child>"
    )
    if expected_topology not in source:
        raise RuntimeError("world_anchor is not fixed directly to world")
    if ("<parent_link>world_anchor</parent_link>" not in source
            or "<child_model>__model__</child_model>" not in source
            or "<child_link>chassis</child_link>" not in source):
        raise RuntimeError("DetachableJoint does not use the intended same-model topology")
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


class TransportPublishers:
    """Keep Gazebo Transport publishers alive for the entire probe.

    Repeated gz topic subprocesses can incur discovery latency and time out
    under scene load. Use native publishers, matching the production
    chassis-lock implementation, while still requiring telemetry confirmation.
    """

    MESSAGE_TYPES = {
        "gz.msgs.Empty": Empty,
        "gz.msgs.Double": Double,
        "gz.msgs.Twist": Twist,
    }

    def __init__(self):
        self.node = gz_transport.Node()
        self.publishers = {}

    def publish(self, topic, typ, payload):
        message_type = self.MESSAGE_TYPES.get(typ)
        if message_type is None:
            raise RuntimeError("unsupported Gazebo message type: " + typ)
        publisher = self.publishers.get(topic)
        if publisher is None:
            publisher = self.node.advertise(topic, message_type)
            if not publisher:
                raise RuntimeError("cannot advertise Gazebo topic " + topic)
            self.publishers[topic] = publisher
        message = message_type()
        if payload.strip():
            text_format.Parse(payload, message)
        if not publisher.publish(message):
            raise RuntimeError("Gazebo Transport publish returned false: " + topic)


_PUBLISHERS = None


def publish(topic, typ, payload):
    if _PUBLISHERS is None:
        raise RuntimeError("Gazebo Transport publishers are not initialized")
    _PUBLISHERS.publish(topic, typ, payload)


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


def wait_for_grasp_carry_plugin(proc, timeout=180.0):
    """Require the production world plugin to be loaded, not merely named in SDF."""
    deadline = time.monotonic() + timeout
    last_log = ""
    while time.monotonic() < deadline:
        ensure_alive(proc, "GraspCarry plugin load")
        try:
            with open(LOG_OUT, encoding="utf-8", errors="replace") as f:
                last_log = f.read()
        except OSError:
            pass
        if PLUGIN_READY_MARKER in last_log:
            return
        if "Failed to load system plugin [libGraspCarry.so]" in last_log:
            raise RuntimeError("Gazebo failed to load production libGraspCarry.so")
        time.sleep(0.05)
    raise RuntimeError(
        "production GraspCarry plugin did not report ready; log_tail="
        + repr(last_log[-1200:])
    )


def passive_stability_challenge(tel, proc, label, window_s=0.35):
    """Mirror startup's no-command pose/stationarity check after each transition."""
    _, origin, samples, _, _ = tel.snapshot()
    if origin is None:
        raise RuntimeError(label + ": missing initial robot pose")
    index = len(samples)
    deadline = time.monotonic() + window_s
    while time.monotonic() < deadline:
        ensure_alive(proc, label + " passive stability")
        time.sleep(0.02)
    _, end, all_samples, _, _ = tel.snapshot()
    window_samples = all_samples[index:]
    if len(window_samples) < 5:
        raise RuntimeError(
            label + ": insufficient passive pose samples (%d)" % len(window_samples)
        )
    xy, yaw = pose_excursion(window_samples, origin)
    result = {
        "phase": label,
        "status": "passed",
        "window_s": window_s,
        "pose_samples": len(window_samples),
        "max_displacement_mm": xy * 1000.0,
        "max_yaw_delta_deg": math.degrees(yaw),
        "start_pose": origin,
        "end_pose": end,
    }
    if xy >= MAX_BASE_DISPLACEMENT_M or yaw >= MAX_BASE_YAW_RAD:
        raise RuntimeError(
            "%s passive pose changed %.3fmm / %.4fdeg"
            % (label, xy * 1000.0, math.degrees(yaw))
        )
    return result


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
    failure = None
    if should_hold and (xy > MAX_BASE_DISPLACEMENT_M or yaw > MAX_BASE_YAW_RAD):
        failure = "attached chassis moved %.3fmm / %.4fdeg" % (
            xy * 1000, math.degrees(yaw))
    if not should_hold and not moved:
        failure = "detached chassis failed to move >=10mm"
    # A failed chassis challenge is evidence, not a control-flow exception:
    # continue the A/B sequence so joint motion can still be tested detached.
    return {"status": "failed" if failure else "passed",
            "error": failure,
            "max_displacement_mm": xy*1000,
            "max_yaw_delta_deg": math.degrees(yaw),
            "start_pose": origin, "end_pose": end,
            "pose_samples": len(all_samples[index:])}


def main():
    global _PUBLISHERS
    _PUBLISHERS = TransportPublishers()
    report = {"result": "FAIL", "joint_stages": [], "state_transitions": [], "drive_challenges": []}
    proc = None
    log = None
    try:
        report["grasp_carry_plugin_retained"] = True
        report["production_external_anchor_removed"] = True
        report["same_model_world_anchor_topology"] = True
        report["sdf_check"] = make_world()
        if not os.path.isfile(PLUGIN_SO):
            raise RuntimeError("built production GraspCarry plugin missing: " + PLUGIN_SO)
        log = open(LOG_OUT, "w", encoding="utf-8")
        env = os.environ.copy()
        plugin_dir = os.path.dirname(PLUGIN_SO)
        search_path = env.get("GZ_SIM_SYSTEM_PLUGIN_PATH", "")
        env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = plugin_dir + (os.pathsep + search_path if search_path else "")
        proc = subprocess.Popen(["gz", "sim", "-v", "4", "-s", "-r", WORLD_OUT],
                                cwd=ROOT, env=env,
                                stdout=log, stderr=subprocess.STDOUT,
                                start_new_session=True)
        wait_for_grasp_carry_plugin(proc)
        report["grasp_carry_plugin_loaded"] = True
        tel = Telemetry()
        wait_for(tel, proc, lambda s: len(s[0]) == len(ARM_JOINTS) and s[1] is not None and s[3] is not None,
                 35, "joint/pose/initial lock telemetry")
        report["initial_state"] = tel.snapshot()[3]
        if tel.snapshot()[3] != "attached":
            raise RuntimeError("internal DetachableJoint did not initialize attached")

        def probe_joint_set(label):
            outcomes = []
            for name, target in ARM_JOINTS.items():
                try:
                    outward = move_joint(tel, proc, name, target)
                    inward = move_joint(tel, proc, name, 0.0)
                    outcomes.append({"joint": name, "status": "passed",
                                     "outward": outward, "reset": inward})
                except Exception as exc:
                    outcomes.append({"joint": name, "status": "failed",
                                     "error": str(exc)})
                    # Continue through the complete six-joint set so one
                    # blocked joint cannot hide independent controller results.
            report[label] = outcomes
            return len(outcomes) == len(ARM_JOINTS) and all(
                item["status"] == "passed" for item in outcomes
            )

        report["passive_stability"] = []
        report["passive_stability"].append(
            passive_stability_challenge(tel, proc, "initial_attached"))
        attached_motion_ok = probe_joint_set("attached_joint_probe")
        report["attached_joint_motion_passed"] = attached_motion_ok
        report["drive_challenges"].append(
            {"phase": "attached", **drive_challenge(tel, proc, True)})

        if not attached_motion_ok:
            # Crucial A/B: the previous probe stopped at the first attached
            # servo timeout and never checked whether the exact same production
            # joints/controllers work after releasing the chassis constraint.
            n = request_state(tel, proc, "detached")
            report["state_transitions"].append({"target": "detached", "attempts": n})
            report["passive_stability"].append(
                passive_stability_challenge(tel, proc, "detached"))
            report["drive_challenges"].append(
                {"phase": "detached", **drive_challenge(tel, proc, False)})
            detached_motion_ok = probe_joint_set("detached_joint_probe")
            report["detached_joint_motion_passed"] = detached_motion_ok
            n = request_state(tel, proc, "attached")
            report["state_transitions"].append({"target": "attached", "attempts": n})
            report["passive_stability"].append(
                passive_stability_challenge(tel, proc, "reattached"))
            report["drive_challenges"].append(
                {"phase": "reattached", **drive_challenge(tel, proc, True)})
            report["diagnostic_complete"] = True
            report["result"] = "FAIL"
            attached_by_joint = {
                item["joint"]: item["status"]
                for item in report.get("attached_joint_probe", [])
            }
            detached_by_joint = {
                item["joint"]: item["status"]
                for item in report.get("detached_joint_probe", [])
            }
            report["joint_motion_summary"] = {
                name: {
                    "attached": attached_by_joint.get(name, "not_run"),
                    "detached": detached_by_joint.get(name, "not_run"),
                }
                for name in ARM_JOINTS
            }
            failed_drive = [
                item for item in report["drive_challenges"]
                if item.get("status") != "passed"
            ]
            if failed_drive and detached_motion_ok:
                report["failure_class"] = (
                    "CHASSIS_HOLD_FAILED_AND_ARM_MOTION_BLOCKED_WHILE_ATTACHED"
                )
            elif failed_drive:
                report["failure_class"] = (
                    "CHASSIS_HOLD_AND_ARM_MOTION_FAILED"
                )
            elif detached_motion_ok:
                report["failure_class"] = "ARM_MOTION_BLOCKED_ONLY_WHILE_ATTACHED"
            else:
                report["failure_class"] = (
                    "ARM_MOTION_BLOCKED_ATTACHED_AND_DETACHED"
                )
            report["state_messages"] = tel.snapshot()[4]
            return 0

        n = request_state(tel, proc, "detached")
        report["state_transitions"].append({"target": "detached", "attempts": n})
        report["passive_stability"].append(
            passive_stability_challenge(tel, proc, "detached"))
        report["drive_challenges"].append(
            {"phase": "detached", **drive_challenge(tel, proc, False)})
        n = request_state(tel, proc, "attached")
        report["state_transitions"].append({"target": "attached", "attempts": n})
        report["passive_stability"].append(
            passive_stability_challenge(tel, proc, "reattached"))
        report["drive_challenges"].append(
            {"phase": "reattached", **drive_challenge(tel, proc, True)})
        n = request_state(tel, proc, "detached")
        report["state_transitions"].append({"target": "detached", "attempts": n})
        report["passive_stability"].append(
            passive_stability_challenge(tel, proc, "final_detached"))
        report["drive_challenges"].append(
            {"phase": "final_detached", **drive_challenge(tel, proc, False)})
        report["state_messages"] = tel.snapshot()[4]
        failed_drive = [item for item in report["drive_challenges"]
                        if item.get("status") != "passed"]
        if failed_drive:
            report["result"] = "FAIL"
            report["failure_class"] = "CHASSIS_HOLD_CHALLENGE_FAILED"
            report["diagnostic_complete"] = True
        else:
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
