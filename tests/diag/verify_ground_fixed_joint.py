#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Verify the independent GroundFixedJoint physical evidence manifest."""
import json
import math
import os
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
SUMMARY = os.path.join(ROOT, "vla_ws", "logs", "ground_fixed_joint_summary.json")


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def main():
    require(os.path.isfile(SUMMARY), "missing GroundFixedJoint summary")
    with open(SUMMARY, encoding="utf-8") as stream:
        summary = json.load(stream)
    results = summary.get("results", [])
    require(summary.get("repeats") == 3 and summary.get("passed") == 3,
            "GroundFixedJoint must report exactly 3/3 passed repeats")
    require(len(results) == 3, "GroundFixedJoint result count is not three")
    for index, result in enumerate(results, 1):
        require(result.get("result") == "PASS", "repeat %d is not PASS" % index)
        require(result.get("gazebo_exit_code") == 0,
                "repeat %d Gazebo did not exit cleanly" % index)
        require(result.get("initial_attached") == "attached" and
                result.get("final_attached") == "detached",
                "repeat %d attach lifecycle incomplete" % index)
        require(result.get("state_messages") ==
                ["attached", "detached", "attached", "detached"],
                "repeat %d state-message sequence invalid" % index)
        stages = {item.get("name"): item for item in result.get("stages", [])}
        require(set(stages) == {"detached_drive", "attached_drive_challenge",
                                "unlock_recovery"},
                "repeat %d stage set incomplete" % index)
        detached = stages["detached_drive"].get("displacement_mm", float("nan"))
        held = stages["attached_drive_challenge"]
        recovery = stages["unlock_recovery"].get("displacement_mm", float("nan"))
        yaw_delta = held.get("yaw_delta_deg", float("nan"))
        wheel_speed = held.get("max_wheel_rad_s", float("nan"))
        require(math.isfinite(detached) and detached >= 10.0,
                "repeat %d detached mobility below 10mm" % index)
        require(math.isfinite(held.get("displacement_mm", float("nan"))) and
                held["displacement_mm"] < 2.0,
                "repeat %d attached displacement is >=2mm" % index)
        require(math.isfinite(yaw_delta) and abs(yaw_delta) < 0.5,
                "repeat %d attached yaw drift is >=0.5deg" % index)
        require(math.isfinite(wheel_speed) and wheel_speed <= 100.0,
                "repeat %d attached wheel-speed guard failed" % index)
        require(math.isfinite(recovery) and recovery >= 10.0,
                "repeat %d unlock recovery below 10mm" % index)
    print("PASS: GroundFixedJoint physical manifest verified (3/3)")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:
        print("FAIL:", exc, file=sys.stderr)
        sys.exit(1)
