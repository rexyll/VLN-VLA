#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Verify that downstream P104 jobs use the exact isolated-stage evidence."""
import hashlib
import json
import math
import os
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
LOCK = os.path.join(ROOT, "vla_ws", "logs",
                    "p104_chassis_brake_stage_locks.json")
EXPECTED = ["world_ready", "baseline_disturbance",
            "locked_hold", "unlock_recovery"]


def sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main():
    if not os.path.isfile(LOCK):
        raise RuntimeError("missing completed isolated-stage lock manifest")
    with open(LOCK, encoding="utf-8") as stream:
        manifest = json.load(stream)

    current_sha = os.environ.get("GITHUB_SHA")
    if current_sha and manifest.get("git_sha") != current_sha:
        raise RuntimeError("stage lock belongs to a different Git commit")

    stages = manifest.get("stages", [])
    names = [stage.get("name") for stage in stages]
    if names != EXPECTED:
        raise RuntimeError("incomplete or out-of-order stage locks: %r" % names)
    if any(stage.get("status") != "passed" for stage in stages):
        raise RuntimeError("one or more isolated stages are not locked as passed")

    hashes = manifest.get("input_hashes", {})
    for name, item in hashes.items():
        path = os.path.join(ROOT, item["path"])
        if not os.path.isfile(path) or sha256(path) != item["sha256"]:
            raise RuntimeError("locked input changed after isolated validation: " + name)

    by_name = {stage["name"]: stage.get("metrics", {}) for stage in stages}
    world_ready = by_name["world_ready"]
    baseline = by_name["baseline_disturbance"]
    locked = by_name["locked_hold"]
    unlock = by_name["unlock_recovery"]
    initial_twist = world_ready.get("twist", [])
    initial_wheels = world_ready.get("wheel_velocities", {})
    if (len(initial_twist) != 3 or
            not all(math.isfinite(value) and abs(value) <= 0.01
                    for value in initial_twist) or
            not initial_wheels or
            not all(math.isfinite(value) and abs(value) <= 0.01
                    for value in initial_wheels.values())):
        raise RuntimeError("world-ready lock does not prove a stationary base")
    for key in ("max_yaw_deg", "max_wheel_rad_s"):
        if not math.isfinite(baseline.get(key, float("nan"))):
            raise RuntimeError("baseline metric is missing or non-finite: " + key)
    if baseline.get("max_yaw_deg", 0.0) < 0.5:
        raise RuntimeError("locked baseline evidence is below 0.5 degree")
    if baseline.get("max_wheel_rad_s", float("inf")) > 100.0:
        raise RuntimeError("locked baseline evidence exceeded 100 rad/s guard")
    if not math.isfinite(locked.get("reduction_ratio", float("nan"))) or locked.get("reduction_ratio", 0.0) < 0.5:
        raise RuntimeError("locked hold evidence did not suppress yaw by 50 percent")
    if not math.isfinite(unlock.get("displacement_mm", float("nan"))) or unlock.get("displacement_mm", 0.0) < 10.0:
        raise RuntimeError("unlock recovery evidence is below 10 mm")

    print("PASS: immutable P104 stage locks verified")
    print("git_sha=%s torque_nm=%s" %
          (manifest.get("git_sha"), manifest.get("candidate_torque_nm")))
    for stage in stages:
        print("LOCKED_STAGE=%s" % stage["name"])
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:
        print("FAIL:", exc, file=sys.stderr)
        sys.exit(1)
