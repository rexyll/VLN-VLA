#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Read-only synchronized Gazebo collector for base / arm coupling experiments.

Run after tools/start_scene.sh, then start this collector in one terminal and
apply exactly one externally controlled experiment condition in another:
  A: arms stationary; B: one arm moved slowly in short segments; C: original
  two-arm motion. Keep /robot/cmd_vel at zero for all three conditions.

This tool never publishes commands or changes the world. It writes JSONL so
samples from independently delivered Gazebo topics retain both receive-time
and simulation-time stamps. Missing model/link/contact telemetry is reported,
not synthesized.
"""
import argparse
import json
import math
import os
import signal
import sys
import threading
import time
from collections import Counter

import gz.transport13 as gz_transport
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.model_pb2 import Model
from gz.msgs10.odometry_pb2 import Odometry
from gz.msgs10.clock_pb2 import Clock
from gz.msgs10.twist_pb2 import Twist
from gz.msgs10.double_pb2 import Double

WORLD = "follow_world"
POSE_TOPIC = f"/world/{WORLD}/dynamic_pose/info"
JOINT_TOPIC = "/world/follow_world/model/robot/joint_state"
ODOM_TOPIC = "/model/robot/odometry"
CLOCK_TOPIC = "/clock"
BASE_CMD_TOPIC = "/model/robot/cmd_vel"
ARM_JOINTS = [
    f"{side}_{joint}_joint"
    for side in ("R", "L")
    for joint in ("pan", "lift", "up_roll", "elbow",
                  "fore_roll", "wrist_p", "wrist_r")
]


def stamp_dict(stamp):
    if stamp is None:
        return None
    sec = int(getattr(stamp, "sec", 0))
    nsec = int(getattr(stamp, "nsec", 0))
    return {"sec": sec, "nanosec": nsec,
            "seconds": sec + nsec * 1e-9}


def vec3(v):
    return {"x": float(v.x), "y": float(v.y), "z": float(v.z)}


def quat(q):
    return {"x": float(q.x), "y": float(q.y),
            "z": float(q.z), "w": float(q.w)}


class Collector:
    def __init__(self, output):
        self.output = output
        self.lock = threading.Lock()
        self.file_lock = threading.Lock()
        self.counts = Counter()
        self.observed_joint_names = set()
        self.latest_sim = None
        self.started = time.monotonic()
        self.stop_at = None
        self.out = open(output, "w", encoding="utf-8", buffering=1)
        self.gz = gz_transport.Node()
        self.subscriptions = []

    def emit(self, source, payload, sim_stamp=None):
        row = {
            "source": source,
            "wall_monotonic_s": time.monotonic() - self.started,
            "sim_time": sim_stamp if sim_stamp is not None else self.latest_sim,
            "payload": payload,
        }
        with self.file_lock:
            self.out.write(json.dumps(row, ensure_ascii=False,
                                      separators=(",", ":")) + "\n")
            self.counts[source] += 1

    def on_clock(self, msg):
        s = stamp_dict(msg.sim)
        if s:
            self.latest_sim = s
        self.emit("clock", {"sim": s}, s)

    def on_pose(self, msg):
        header_stamp = None
        if msg.HasField("header") and msg.header.HasField("stamp"):
            header_stamp = stamp_dict(msg.header.stamp)
        wanted = {"robot", "robot::chassis", "robot::left_wheel",
                  "robot::right_wheel", "chassis"}
        poses = []
        names = []
        for p in msg.pose:
            names.append(p.name)
            if p.name in wanted:
                poses.append({
                    "name": p.name,
                    "position": vec3(p.position),
                    "orientation": quat(p.orientation),
                })
        self.emit("dynamic_pose", {
            "header_stamp": header_stamp,
            "entity_count": len(msg.pose),
            "entity_names_sample": names[:30],
            "watched_poses": poses,
        }, header_stamp)

    def on_joint(self, msg):
        header_stamp = None
        if msg.HasField("header") and msg.header.HasField("stamp"):
            header_stamp = stamp_dict(msg.header.stamp)
        joints = []
        for j in msg.joint:
            if (j.name in ("left_wheel_joint", "right_wheel_joint")
                    or j.name.startswith(("R_", "L_"))):
                item = {"name": j.name}
                for axis_name in ("axis1", "axis2"):
                    if j.HasField(axis_name):
                        a = getattr(j, axis_name)
                        vals = {}
                        for field in ("position", "velocity", "force"):
                            # Axis scalar fields are proto3 scalars; only axis1 /
                            # axis2 submessages have presence semantics.
                            value = float(getattr(a, field))
                            if math.isfinite(value):
                                vals[field] = value
                        if vals:
                            item[axis_name] = vals
                joints.append(item)
        with self.lock:
            self.observed_joint_names.update(joint["name"] for joint in joints)
        self.emit("joint_state", {
            "topic": JOINT_TOPIC,
            "header_stamp": header_stamp,
            "model_name": msg.name,
            "joint_count": len(msg.joint),
            "watched_joints": joints,
        }, header_stamp)

    def on_odom(self, msg):
        header_stamp = None
        if msg.HasField("header") and msg.header.HasField("stamp"):
            header_stamp = stamp_dict(msg.header.stamp)
        self.emit("wheel_odometry", {
            "header_stamp": header_stamp,
            "position": vec3(msg.pose.position),
            "orientation": quat(msg.pose.orientation),
            "linear_velocity": vec3(msg.twist.linear),
            "angular_velocity": vec3(msg.twist.angular),
        }, header_stamp)

    def on_base_cmd(self, msg):
        self.emit("base_cmd_vel", {
            "linear": vec3(msg.linear),
            "angular": vec3(msg.angular),
        })

    def on_arm_cmd(self, joint, msg):
        # Keep per-topic coverage distinct: an aggregate source counter cannot
        # reveal which of the 14 target topics actually delivered messages.
        self.emit(f"arm_position_target:{joint}", {
            "joint": joint, "target": float(msg.data),
        })

    def subscribe(self, msg_type, topic, callback):
        ok = self.gz.subscribe(msg_type, topic, callback)
        if not ok:
            raise RuntimeError(f"Gazebo subscription failed: {topic}")
        self.subscriptions.append((topic, callback))
        print(f"subscribed: {topic}")

    def close(self):
        self.out.flush()
        self.out.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=45.0,
                        help="collection duration in seconds (default: 45)")
    parser.add_argument("--output", default=None,
                        help="JSONL output path")
    args = parser.parse_args()
    if args.duration <= 0:
        parser.error("--duration must be > 0")

    output = args.output or os.path.join(
        "vla_ws", "logs",
        f"base_arm_dynamics_{time.strftime('%Y%m%d_%H%M%S')}.jsonl")
    os.makedirs(os.path.dirname(os.path.abspath(output)), exist_ok=True)
    c = Collector(output)
    try:
        c.subscribe(Clock, CLOCK_TOPIC, c.on_clock)
        c.subscribe(Pose_V, POSE_TOPIC, c.on_pose)
        c.subscribe(Model, JOINT_TOPIC, c.on_joint)
        c.subscribe(Odometry, ODOM_TOPIC, c.on_odom)
        c.subscribe(Twist, BASE_CMD_TOPIC, c.on_base_cmd)
        for joint in ARM_JOINTS:
            c.subscribe(Double, f"/drink/cmd/{joint}",
                        lambda msg, name=joint: c.on_arm_cmd(name, msg))
        print(f"\nRead-only capture for {args.duration:.1f}s")
        print("Do not combine A/B/C conditions in one file; run a fresh capture per condition.")
        deadline = time.monotonic() + args.duration
        while time.monotonic() < deadline:
            time.sleep(min(0.1, max(0.0, deadline - time.monotonic())))
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        counts = dict(c.counts)
        c.close()
        print(f"\nJSONL: {os.path.abspath(output)}")
        print("records:", json.dumps(counts, ensure_ascii=False, sort_keys=True))
        with c.lock:
            observed_joint_names = sorted(c.observed_joint_names)
        print("observed_joint_names:", json.dumps(
            observed_joint_names, ensure_ascii=False))
        required_joint_names = {"left_wheel_joint", "right_wheel_joint", *ARM_JOINTS}
        missing_joints = sorted(required_joint_names - set(observed_joint_names))
        if missing_joints:
            print("MISSING REQUIRED JOINTS:", ", ".join(missing_joints))
        missing = [name for name in
                   ("clock", "dynamic_pose", "joint_state", "wheel_odometry",
                    "base_cmd_vel") if not counts.get(name)]
        if missing:
            print("MISSING REQUIRED SOURCES:", ", ".join(missing))
            return 2
        if missing_joints:
            return 2
        if not counts.get("arm_position_target"):
            print("WARNING: no arm target messages observed during capture")
        return 0


if __name__ == "__main__":
    sys.exit(main())
