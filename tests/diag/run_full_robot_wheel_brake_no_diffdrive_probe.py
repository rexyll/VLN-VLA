#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Isolate full-robot arm reaction with DiffDrive removed from diagnostic copy.\n\nThe diagnostic brake cap and copied wheel-joint effort limit are selected by
VLA_DIAG_BRAKE_TORQUE_NM and VLA_DIAG_WHEEL_EFFORT_NM. Production SDF and wheel
limits remain unchanged; this sweeps brake authority without changing the
production robot.

This creates a diagnostic copy of production work_area.sdf, removes only the
external chassis anchor in that copy, and tests whether a stronger wheel brake
can resist arm reaction while preserving the production robot and controllers.
No production SDF is modified.
"""
import json
import math
import os
import re
import signal
import sys
import subprocess
import threading
import time

import gz.transport13 as gz_transport
from google.protobuf import text_format
from gz.msgs10.double_pb2 import Double
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.empty_pb2 import Empty
from gz.msgs10.model_pb2 import Model
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.stringmsg_pb2 import StringMsg
from gz.msgs10.twist_pb2 import Twist

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
SOURCE = os.path.join(ROOT, "vla_ws/src/vla_follow/worlds/work_area.sdf")
LOG_DIR = os.path.join(ROOT, "vla_ws/logs")
BRAKE_COMMAND_TOPIC = "/chassis_brake/enable"
BRAKE_STATUS_TOPIC = "/chassis_brake/status"
BRAKE_MAX_TORQUE_NM = float(os.environ.get("VLA_DIAG_BRAKE_TORQUE_NM", "160"))
DIAGNOSTIC_WHEEL_EFFORT_NM = float(os.environ.get("VLA_DIAG_WHEEL_EFFORT_NM", "160"))
BRAKE_POSITION_KP = float(os.environ.get("VLA_DIAG_BRAKE_KP", "80"))
BRAKE_VELOCITY_KD = float(os.environ.get("VLA_DIAG_BRAKE_KD", "2"))
if (not all(math.isfinite(v) for v in (BRAKE_MAX_TORQUE_NM, DIAGNOSTIC_WHEEL_EFFORT_NM, BRAKE_POSITION_KP, BRAKE_VELOCITY_KD))
        or BRAKE_MAX_TORQUE_NM <= 0 or DIAGNOSTIC_WHEEL_EFFORT_NM < BRAKE_MAX_TORQUE_NM
        or BRAKE_MAX_TORQUE_NM > 200 or BRAKE_POSITION_KP < 0 or BRAKE_VELOCITY_KD < 0):
    raise ValueError("invalid diagnostic brake torque/effort/gains")
TORQUE_TAG = (f"{BRAKE_MAX_TORQUE_NM:g}").replace(".", "p")
KP_TAG = (f"{BRAKE_POSITION_KP:g}").replace(".", "p")
KD_TAG = (f"{BRAKE_VELOCITY_KD:g}").replace(".", "p")
GAIN_TAG = "t%s_kp%s_kd%s" % (TORQUE_TAG, KP_TAG, KD_TAG)
WORLD_OUT = os.path.join(LOG_DIR, "full_robot_wheel_brake_no_diffdrive_%s.sdf" % GAIN_TAG)
LOG_OUT = os.path.join(LOG_DIR, "full_robot_wheel_brake_no_diffdrive_%s_gz.log" % GAIN_TAG)
REPORT_OUT = os.path.join(LOG_DIR, "full_robot_wheel_brake_no_diffdrive_%s.json" % GAIN_TAG)
BRAKE_PLUGIN_SO = os.path.join(ROOT, "vla_ws/src/vla_follow/gz_plugins/build/libChassisBrake.so")
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
DRIVE_JOINTS = ("left_wheel_joint", "right_wheel_joint")
MAX_BASE_DISPLACEMENT_M = 0.002
MAX_BASE_YAW_RAD = math.radians(0.5)
PLUGIN_SO = os.path.join(ROOT, "vla_ws/src/vla_follow/gz_plugins/build/libGraspCarry.so")
PLUGIN_READY_MARKER = "[GraspCarry] 就绪"
BRAKE_READY_MARKER = "[ChassisBrake] Ready;"


def make_world():
    """Create a diagnostic copy: remove only the world anchor and add wheel brake to robot."""
    with open(SOURCE, encoding="utf-8") as stream:
        source = stream.read()
    anchor = re.compile(
        r"\s*<!-- Physical chassis immobilizer[\s\S]*?</model>\s*",
        re.MULTILINE)
    source, removed = anchor.subn("\n", source, count=1)
    if removed != 1:
        raise RuntimeError("expected exactly one production world-rooted chassis anchor")
    # Remove DiffDrive only from this disposable diagnostic copy. This A/B
    # isolation determines whether its wheel velocity controller competes with
    # the brake during arm articulation; production SDF remains untouched.
    diffdrive = re.compile(
        r'\s*<plugin\s+filename="gz-sim-diff-drive-system"'
        r'[\s\S]*?</plugin>\s*', re.MULTILINE)
    source, diffdrive_removed = diffdrive.subn("\n", source, count=1)
    if diffdrive_removed != 1:
        raise RuntimeError("expected exactly one DiffDrive plugin in diagnostic copy")
    # In this disposable diagnostic copy only, raise both drive-wheel effort
    # limits with the brake cap. Otherwise the existing 5 N m joint effort
    # ceiling can mask whether the brake controller needs more authority.
    for joint_name in ("left_wheel_joint", "right_wheel_joint"):
        # Match one complete SDF joint block and verify the resulting limit.
        pattern = (r'(<joint\s+name="' + re.escape(joint_name) +
                   r'"[\s\S]*?<effort>)\s*5(?:\.0)?\s*(</effort>)')
        source, changed = re.subn(
            pattern, r"\g<1>%.9g\g<2>" % DIAGNOSTIC_WHEEL_EFFORT_NM,
            source, count=1)
        if changed != 1:
            raise RuntimeError("could not raise diagnostic effort for " + joint_name)
        joint = re.search(
            r'<joint\s+name="' + re.escape(joint_name) + r'"[\s\S]*?</joint>',
            source)
        effort = (re.search(r'<effort>\s*([0-9.eE+-]+)\s*</effort>', joint.group(0))
                  if joint else None)
        if effort is None or not math.isclose(
                float(effort.group(1)), DIAGNOSTIC_WHEEL_EFFORT_NM, rel_tol=0, abs_tol=1e-9):
            raise RuntimeError("diagnostic effort verification failed for " + joint_name)
    marker = "<!-- DRINK:END robot_ext -->\n    </model>"
    plugin = """<!-- Diagnostic-only wheel brake; not added to production work_area.sdf. -->
      <plugin filename="libChassisBrake.so" name="vla::ChassisBrake">
        <left_joint>left_wheel_joint</left_joint>
        <right_joint>right_wheel_joint</right_joint>
        <command_topic>/chassis_brake/enable</command_topic>
        <status_topic>/chassis_brake/status</status_topic>
        <position_kp>%.9g</position_kp>
        <velocity_kd>%.9g</velocity_kd>
        <max_torque>%.9g</max_torque>
      </plugin>
    """ % (BRAKE_POSITION_KP, BRAKE_VELOCITY_KD, BRAKE_MAX_TORQUE_NM)
    if marker not in source:
        raise RuntimeError("production robot model extension marker changed")
    source = source.replace(marker, "<!-- DRINK:END robot_ext -->\n    " + plugin + "</model>", 1)
    if "ground_lock_anchor" in source or "gz-sim-detachable-joint-system" in source:
        raise RuntimeError("diagnostic world still contains the old detachable anchor")
    if source.count('name="vla::ChassisBrake"') != 1:
        raise RuntimeError("diagnostic wheel brake plugin insertion is not unique")
    os.makedirs(LOG_DIR, exist_ok=True)
    with open(WORLD_OUT, "w", encoding="utf-8") as stream:
        stream.write(source)
    check = subprocess.run(
        ["gz", "sdf", "-k", WORLD_OUT], cwd=ROOT, text=True,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=30)
    if check.returncode:
        raise RuntimeError("wheel-brake production-world diagnostic SDF invalid:\n" + check.stdout)
    return check.stdout[-3000:]

class Telemetry:
    def __init__(self):
        self.lock = threading.Lock()
        self.joints = {}
        self.wheels = {}
        self.wheel_samples = []
        self.pose = None
        self.samples = []
        self.brake_status = None
        self.brake_messages = []
        self.node = gz_transport.Node()
        for typ, topic, cb in (
            (Model, JOINT_TOPIC, self.on_joints),
            (Pose_V, POSE_TOPIC, self.on_pose),
            (StringMsg, BRAKE_STATUS_TOPIC, self.on_brake_status),
        ):
            if not self.node.subscribe(typ, topic, cb):
                raise RuntimeError("cannot subscribe " + topic)

    def on_joints(self, msg):
        values = {}
        wheel_values = {}
        for j in msg.joint:
            if j.name in ARM_JOINTS and j.HasField("axis1"):
                values[j.name] = (float(j.axis1.position), float(j.axis1.velocity))
            elif j.name in DRIVE_JOINTS and j.HasField("axis1"):
                wheel_values[j.name] = (float(j.axis1.position), float(j.axis1.velocity))
        with self.lock:
            self.joints.update(values)
            self.wheels.update(wheel_values)
            if wheel_values:
                self.wheel_samples.append((time.monotonic(), dict(self.wheels)))

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

    def on_brake_status(self, msg):
        status = str(msg.data).strip()
        with self.lock:
            self.brake_status = status
            self.brake_messages.append(status)

    def snapshot(self):
        with self.lock:
            return (dict(self.joints), self.pose, list(self.samples),
                    self.brake_status, list(self.brake_messages))

    def wheel_snapshot(self):
        with self.lock:
            return dict(self.wheels), list(self.wheel_samples)

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
        "gz.msgs.Boolean": Boolean,
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


def publish_brake_command(enabled):
    """Use the CLI transport path for the brake command, matching the proven isolated probe."""
    result = subprocess.run(
        ["gz", "topic", "-t", BRAKE_COMMAND_TOPIC, "-m", "gz.msgs.Boolean",
         "-p", "data: %s" % ("true" if enabled else "false")],
        cwd=ROOT, text=True, stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT, timeout=10)
    if result.returncode:
        raise RuntimeError("ChassisBrake command publish failed: " + result.stdout)


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
    # A one-shot Gazebo Transport publish can race subscriber discovery when
    # this is the first command sent to a joint topic in a fresh world. Repeat
    # the setpoint at 10 Hz until measured motion/settling confirms delivery.
    topic = "/drink/cmd/" + name
    payload = "data: %.6f" % target
    deadline = time.monotonic() + 8.0
    stable = None
    peak = 0.0
    last = None
    next_publish = 0.0
    while time.monotonic() < deadline:
        ensure_alive(proc, "move " + name)
        now = time.monotonic()
        if now >= next_publish:
            publish(topic, "gz.msgs.Double", payload)
            next_publish = now + 0.1
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
    raise RuntimeError("%s target %.3f did not settle; last=%r peak_velocity=%.6f" %
                       (name, target, last, peak))


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
        if PLUGIN_READY_MARKER in last_log and BRAKE_READY_MARKER in last_log:
            return
        if "Failed to load system plugin [libGraspCarry.so]" in last_log:
            raise RuntimeError("Gazebo failed to load production libGraspCarry.so")
        time.sleep(0.05)
    raise RuntimeError(
        "production GraspCarry / diagnostic ChassisBrake plugins did not report ready; log_tail="
        + repr(last_log[-1200:])
    )


def passive_stability_challenge(tel, proc, label, window_s=0.35, require_stable=True):
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
        message = "%s passive pose changed %.3fmm / %.4fdeg" % (
            label, xy * 1000.0, math.degrees(yaw))
        result["status"] = "failed"
        result["error"] = message
        if require_stable:
            raise RuntimeError(message)
    return result


def pose_excursion(samples, origin):
    xy = yaw = 0.0
    for _, p in samples:
        xy = max(xy, math.hypot(p[0]-origin[0], p[1]-origin[1]))
        dyaw = math.atan2(math.sin(p[3]-origin[3]), math.cos(p[3]-origin[3]))
        yaw = max(yaw, abs(dyaw))
    return xy, yaw


def wheel_excursion(samples, origin):
    """Return maximum absolute drive-wheel angle and speed during a probe."""
    result = {}
    for name in DRIVE_JOINTS:
        start = origin.get(name)
        if start is None:
            result[name] = {"available": False}
            continue
        max_angle = max_speed = 0.0
        count = 0
        for _, values in samples:
            state = values.get(name)
            if state is None:
                continue
            count += 1
            max_angle = max(max_angle, abs(state[0] - start[0]))
            max_speed = max(max_speed, abs(state[1]))
        result[name] = {
            "available": count > 0,
            "samples": count,
            "max_abs_angle_delta_rad": max_angle,
            "max_abs_velocity_rad_s": max_speed,
        }
    return result


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
    report = {
        "result": "FAIL",
        "diagnostic_only_world_copy": True,
        "production_world_modified": False,
        "old_external_anchor_removed_from_copy": True,
        "diff_drive_plugin_removed_from_copy": True,
        "brake_max_torque_nm": BRAKE_MAX_TORQUE_NM,
        "brake_position_kp": BRAKE_POSITION_KP,
        "brake_velocity_kd": BRAKE_VELOCITY_KD,
        "diagnostic_wheel_effort_limit_nm": DIAGNOSTIC_WHEEL_EFFORT_NM,
        "arm_joint_probes": [],
        "base_challenges": [],
        "stability_checks": [],
    }
    proc = None
    log = None
    try:
        report["sdf_check"] = make_world()
        if not os.path.isfile(PLUGIN_SO):
            raise RuntimeError("built production GraspCarry plugin missing: " + PLUGIN_SO)
        if not os.path.isfile(BRAKE_PLUGIN_SO):
            raise RuntimeError("built ChassisBrake plugin missing: " + BRAKE_PLUGIN_SO)
        log = open(LOG_OUT, "w", encoding="utf-8")
        env = os.environ.copy()
        plugin_dir = os.path.dirname(PLUGIN_SO)
        search_path = env.get("GZ_SIM_SYSTEM_PLUGIN_PATH", "")
        env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = plugin_dir + (
            os.pathsep + search_path if search_path else "")
        proc = subprocess.Popen(
            ["gz", "sim", "-v", "4", "-s", "-r", WORLD_OUT],
            cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT,
            start_new_session=True)
        wait_for_grasp_carry_plugin(proc)
        report["grasp_carry_plugin_loaded"] = True
        report["chassis_brake_plugin_loaded"] = True
        tel = Telemetry()
        wait_for(tel, proc,
                 lambda snap: len(snap[0]) == len(ARM_JOINTS) and snap[1] is not None,
                 35, "full production robot joint/pose telemetry")

        # Engage wheel brake and require its explicit plugin acknowledgement.
        publish_brake_command(True)
        deadline = time.monotonic() + 6.0
        while time.monotonic() < deadline:
            ensure_alive(proc, "wheel brake lock acknowledgement")
            if tel.snapshot()[3] and tel.snapshot()[3].startswith("LOCKED"):
                break
            time.sleep(0.02)
        else:
            raise RuntimeError("full robot ChassisBrake did not acknowledge LOCKED: %r" %
                               tel.snapshot()[4][-10:])
        report["brake_lock_ack"] = tel.snapshot()[3]

        def record_stability(label, start_index, origin):
            _, _, samples, _, _ = tel.snapshot()
            window = samples[start_index:]
            if len(window) < 5:
                raise RuntimeError(label + " insufficient base pose samples")
            xy, yaw = pose_excursion(window, origin)
            result = {
                "phase": label,
                "max_displacement_mm": xy * 1000.0,
                "max_yaw_delta_deg": math.degrees(yaw),
                "pose_samples": len(window),
                "status": "passed" if xy < MAX_BASE_DISPLACEMENT_M and
                          yaw < MAX_BASE_YAW_RAD else "failed",
            }
            report["stability_checks"].append(result)
            if result["status"] != "passed":
                raise RuntimeError("%s base drift %.3fmm / %.4fdeg" %
                                   (label, xy * 1000.0, math.degrees(yaw)))
            return result

        # A short passive hold first, then articulate every tested production arm
        # joint while the chassis wheel brake remains enabled.
        _, origin, samples, _, _ = tel.snapshot()
        passive_index = len(samples)
        time.sleep(0.5)
        record_stability("brake_passive_hold", passive_index, origin)

        for name, target in ARM_JOINTS.items():
            ensure_alive(proc, "arm probe " + name)
            # Establish a fresh measured zero before movement, not just a
            # controller command acknowledgement.
            publish("/drink/cmd/" + name, "gz.msgs.Double", "data: 0.0")
            deadline = time.monotonic() + 5.0
            baseline = None
            while time.monotonic() < deadline:
                ensure_alive(proc, "arm zero baseline " + name)
                baseline = tel.snapshot()[0].get(name)
                if baseline and abs(baseline[0]) < 0.02 and abs(baseline[1]) < 0.08:
                    break
                time.sleep(0.02)
            else:
                raise RuntimeError("%s zero baseline not confirmed: %r" % (name, baseline))
            _, base_origin, pose_samples, _, _ = tel.snapshot()
            pose_index = len(pose_samples)
            wheel_origin, wheel_samples = tel.wheel_snapshot()
            wheel_index = len(wheel_samples)
            if not all(wheel_origin.get(j) for j in DRIVE_JOINTS):
                raise RuntimeError("drive-wheel joint telemetry unavailable before arm probe")
            outward = move_joint(tel, proc, name, target)
            inward = move_joint(tel, proc, name, 0.0)
            _, _, after_samples, _, _ = tel.snapshot()
            _, after_wheel_samples = tel.wheel_snapshot()
            arm_xy, arm_yaw = pose_excursion(after_samples[pose_index:], base_origin)
            wheel_motion = wheel_excursion(
                after_wheel_samples[wheel_index:], wheel_origin)
            item = {
                "joint": name, "status": "passed",
                "outward": outward, "reset": inward,
                "base_max_displacement_mm": arm_xy * 1000.0,
                "base_max_yaw_delta_deg": math.degrees(arm_yaw),
                "drive_wheel_motion": wheel_motion,
            }
            if arm_xy >= MAX_BASE_DISPLACEMENT_M or arm_yaw >= MAX_BASE_YAW_RAD:
                item["status"] = "failed"
            report["arm_joint_probes"].append(item)
            # Collect all six joints even when the first joint exceeds the
            # chassis envelope; this is diagnostic evidence, not acceptance.

        # A/B control: repeat R_lift movement with the brake released. Since
        # DiffDrive was removed from this disposable world, this isolates the
        # wheel-brake contribution without controller competition.
        publish_brake_command(False)
        deadline = time.monotonic() + 6.0
        while time.monotonic() < deadline:
            ensure_alive(proc, "wheel brake unlock acknowledgement")
            if tel.snapshot()[3] and tel.snapshot()[3].startswith("UNLOCKED"):
                break
            time.sleep(0.02)
        else:
            raise RuntimeError("full robot ChassisBrake did not acknowledge UNLOCKED")
        unlocked = []
        for target in (-0.15, 0.0):
            _, base_origin, pose_samples, _, _ = tel.snapshot()
            pose_index = len(pose_samples)
            wheel_origin, wheel_samples = tel.wheel_snapshot()
            wheel_index = len(wheel_samples)
            move_joint(tel, proc, "R_lift_joint", target)
            _, _, after_samples, _, _ = tel.snapshot()
            _, after_wheel_samples = tel.wheel_snapshot()
            xy, yaw = pose_excursion(after_samples[pose_index:], base_origin)
            unlocked.append({
                "target_rad": target,
                "base_max_displacement_mm": xy * 1000.0,
                "base_max_yaw_delta_deg": math.degrees(yaw),
                "drive_wheel_motion": wheel_excursion(
                    after_wheel_samples[wheel_index:], wheel_origin),
            })
        report["brake_unlocked_arm_ab"] = unlocked
        report["result"] = "DATA_COLLECTED"
        report["diagnostic_complete"] = True
        report["scope"] = "six arm joints locked-brake plus R_lift unlocked-brake A/B; DiffDrive absent"
        return 0

        # Challenge DiffDrive while brake is active. The same challenge must
        # move the chassis after the brake is removed.
        held = drive_challenge(tel, proc, True)
        report["base_challenges"].append({"phase": "brake_locked", **held})
        if held["status"] != "passed":
            raise RuntimeError("wheel brake failed full-robot drive challenge: " +
                               str(held["error"]))

        publish_brake_command(False)
        deadline = time.monotonic() + 6.0
        while time.monotonic() < deadline:
            ensure_alive(proc, "wheel brake unlock acknowledgement")
            if tel.snapshot()[3] and tel.snapshot()[3].startswith("UNLOCKED"):
                break
            time.sleep(0.02)
        else:
            raise RuntimeError("full robot ChassisBrake did not acknowledge UNLOCKED: %r" %
                               tel.snapshot()[4][-10:])
        report["brake_unlock_ack"] = tel.snapshot()[3]
        released = drive_challenge(tel, proc, False)
        report["base_challenges"].append({"phase": "brake_unlocked", **released})
        if released["status"] != "passed":
            raise RuntimeError("DiffDrive did not recover after brake release: " +
                               str(released["error"]))

        # Re-lock to prove the same full-robot assembly can return to a stable hold.
        publish_brake_command(True)
        deadline = time.monotonic() + 6.0
        while time.monotonic() < deadline:
            ensure_alive(proc, "wheel brake relock acknowledgement")
            if tel.snapshot()[3] and tel.snapshot()[3].startswith("LOCKED"):
                break
            time.sleep(0.02)
        else:
            raise RuntimeError("full robot ChassisBrake did not re-lock")
        _, origin, samples, _, _ = tel.snapshot()
        index = len(samples)
        time.sleep(0.5)
        record_stability("brake_relocked_hold", index, origin)

        report["result"] = "PASS"
        report["diagnostic_complete"] = True
        return 0
    except Exception as exc:
        report["error"] = str(exc)
        report["diagnostic_complete"] = False
        return 1
    finally:
        if proc is not None:
            try:
                publish_brake_command(False)
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
        if log is not None:
            log.close()
        os.makedirs(LOG_DIR, exist_ok=True)
        with open(REPORT_OUT, "w", encoding="utf-8") as stream:
            json.dump(report, stream, indent=2, sort_keys=True)
            stream.write("\n")
        print(json.dumps(report, indent=2, sort_keys=True))


if __name__ == "__main__":
    sys.exit(main())

