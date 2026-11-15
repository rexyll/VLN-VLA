#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Offline, read-only summary for collect_base_arm_dynamics.py JSONL captures.

This reports observed telemetry only. It deliberately defines no pass/fail
thresholds and makes no causal claims about arm/base coupling.
"""
import argparse
import json
import math
import statistics
from collections import defaultdict


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q["w"] * q["z"] + q["x"] * q["y"]),
        1.0 - 2.0 * (q["y"] * q["y"] + q["z"] * q["z"]),
    )


def load_rows(path):
    rows = []
    with open(path, encoding="utf-8") as stream:
        for line_no, line in enumerate(stream, 1):
            if not line.strip():
                continue
            try:
                row = json.loads(line)
            except json.JSONDecodeError as exc:
                raise ValueError(f"invalid JSON at line {line_no}: {exc}") from exc
            if not isinstance(row, dict) or "source" not in row:
                raise ValueError(f"line {line_no}: expected object with source")
            rows.append(row)
    return rows


def percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, math.ceil(fraction * len(ordered)) - 1))
    return ordered[index]


def analyze(rows):
    by_source = defaultdict(list)
    for row in rows:
        by_source[str(row["source"])].append(row)

    coverage = {}
    for source, items in sorted(by_source.items()):
        stamps = [
            item.get("sim_time", {}).get("seconds")
            for item in items
            if isinstance(item.get("sim_time"), dict)
            and isinstance(item["sim_time"].get("seconds"), (int, float))
        ]
        coverage[source] = {
            "records": len(items),
            "sim_timestamped_records": len(stamps),
            "sim_time_start_s": min(stamps) if stamps else None,
            "sim_time_end_s": max(stamps) if stamps else None,
        }

    pose_tracks = defaultdict(list)
    for row in by_source.get("dynamic_pose", []):
        payload = row.get("payload") or {}
        stamp = (row.get("sim_time") or {}).get("seconds")
        for pose in payload.get("watched_poses", []):
            if pose.get("name") in ("robot", "robot::chassis"):
                p = pose.get("position") or {}
                q = pose.get("orientation") or {}
                if all(k in p for k in ("x", "y", "z")) and all(
                    k in q for k in ("x", "y", "z", "w")
                ):
                    pose_tracks[pose["name"]].append(
                        (stamp, p["x"], p["y"], p["z"], yaw_from_quaternion(q))
                    )

    pose_summary = {}
    for name, samples in sorted(pose_tracks.items()):
        first = samples[0]
        displacements = [
            math.sqrt((s[1] - first[1]) ** 2 + (s[2] - first[2]) ** 2)
            for s in samples
        ]
        yaw_deltas = [
            abs(math.atan2(math.sin(s[4] - first[4]), math.cos(s[4] - first[4])))
            for s in samples
        ]
        pose_summary[name] = {
            "samples": len(samples),
            "max_xy_displacement_from_first_m": max(displacements),
            "max_abs_yaw_change_from_first_deg": math.degrees(max(yaw_deltas)),
            "first_sim_time_s": first[0],
            "last_sim_time_s": samples[-1][0],
        }

    wheel_velocities, arm_velocities, base_cmd_magnitudes = [], [], []
    joint_names = set()
    for row in by_source.get("joint_state", []):
        for joint in (row.get("payload") or {}).get("watched_joints", []):
            name = joint.get("name", "")
            joint_names.add(name)
            target = wheel_velocities if "wheel" in name else arm_velocities
            for axis in ("axis1", "axis2"):
                velocity = (joint.get(axis) or {}).get("velocity")
                if isinstance(velocity, (int, float)) and math.isfinite(velocity):
                    target.append(abs(float(velocity)))
    for row in by_source.get("base_cmd_vel", []):
        payload = row.get("payload") or {}
        linear = payload.get("linear") or {}
        angular = payload.get("angular") or {}
        vals = [abs(float((linear.get(k) or 0.0))) for k in ("x", "y", "z")]
        vals += [abs(float((angular.get(k) or 0.0))) for k in ("x", "y", "z")]
        base_cmd_magnitudes.append(max(vals))

    def range_stats(values):
        return {
            "samples": len(values),
            "max_abs": max(values) if values else None,
            "p95_abs": percentile(values, 0.95),
        }

    return {
        "record_count": len(rows),
        "source_coverage": coverage,
        "pose_displacement": pose_summary,
        "joint_velocity_abs_rad_s": {
            "wheel": range_stats(wheel_velocities),
            "arm": range_stats(arm_velocities),
            "observed_joint_names": sorted(joint_names),
        },
        "base_command_abs_components": range_stats(base_cmd_magnitudes),
        "interpretation_limits": [
            "This is descriptive telemetry, not a pass/fail evaluation.",
            "Missing chassis pose or contact telemetry is reported as missing; it is not inferred.",
            "JointState force values are not treated as proof of brake torque.",
            "Correlation between arm motion and base motion does not establish causation.",
            "Compare A/B/C only when scene, initial state, physics and simulated-time windows are controlled.",
        ],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("jsonl", help="JSONL file emitted by the read-only collector")
    parser.add_argument("--output", help="optional path for JSON summary")
    args = parser.parse_args()
    result = analyze(load_rows(args.jsonl))
    rendered = json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True)
    if args.output:
        with open(args.output, "w", encoding="utf-8") as stream:
            stream.write(rendered + "\n")
    print(rendered)


if __name__ == "__main__":
    main()
