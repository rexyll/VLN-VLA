# Base–arm dynamics diagnostic runbook

This procedure collects evidence for the PREPARE/PRESENT chassis-drift investigation. It is deliberately **read-only**: the collector subscribes to Gazebo Transport topics and writes JSONL; it does not publish motion commands, call Gazebo services, or change physics.

## Safety and experiment controls

- Run only in the disposable Gazebo simulation world, never on a connected physical robot.
- Keep the production state machine and all safety gates unchanged.
- Do not change wheel friction, damping, mass, controller gains, joint limits, ARM_VMAX, collision checks, IK/grasp tolerances, handover limits, or release conditions during these captures.
- Keep the base command at zero in all three groups. Confirm the command topic is zero from recorded samples; do not assume it.
- Use a fresh scene reset and a separate output file for each group. Do not compare captures unless world, initial robot pose, simulation step/configuration, controller setup, and capture duration are equivalent.
- The collector does not drive the arm. An operator must apply the specified arm condition through the existing, approved simulation controls. If no safe, repeatable way to apply a condition is available, stop rather than improvise a new command publisher.

## Conditions

| Group | Arm condition | Base condition | Purpose |
|---|---|---|---|
| A — baseline | Both arms stationary at the same known initial pose | Zero command | Measure passive base drift and telemetry noise |
| B — single arm | Move only one arm slowly in short, separated segments; pause between segments | Zero command | Isolate single-arm motion correlation |
| C — original motion | Reproduce the original two-arm PREPARE motion without changing its trajectory | Zero command | Compare the original coupled-motion case |

For each group:
1. Reset and start the same `follow_world` scene from the same initial robot pose.
2. Confirm Gazebo Transport and ROS 2 bridge are running and inspect topic availability.
3. Start the collector, then perform only that group's arm condition.
4. Keep collecting through the full motion and a short stationary settling interval.
5. Stop the collector cleanly and preserve the raw JSONL unchanged.

## Capture

From the repository root, in the ROS 2 Jazzy environment with the Gazebo Transport Python bindings available:

```bash
source /opt/ros/jazzy/setup.bash
python3 tests/diag/collect_base_arm_dynamics.py --duration 45 --output /tmp/base-arm-A.jsonl
```

Use distinct names for B and C, for example `/tmp/base-arm-B.jsonl` and `/tmp/base-arm-C.jsonl`. Start a new collector process for each reset/condition. The default duration is 45 seconds; choose the same duration for all groups.

Before interpreting a capture, verify that it contains records for:
- `clock`
- `dynamic_pose`
- `joint_state`
- `odometry`
- `base_cmd_vel`
- the 14 arm target topics (R/L pan, lift, up_roll, elbow, fore_roll, wrist_p, wrist_r)

The collector reports missing sources and exits nonzero rather than synthesizing missing data. The dynamic-pose stream may not expose `robot::chassis`; record that as unavailable rather than inferring a model/link transform. Contact-state telemetry is not currently collected by this tool.

## Offline analysis

```bash
python3 tests/diag/analyze_base_arm_dynamics.py /tmp/base-arm-A.jsonl --output /tmp/base-arm-A-summary.json
python3 tests/diag/analyze_base_arm_dynamics.py /tmp/base-arm-B.jsonl --output /tmp/base-arm-B-summary.json
python3 tests/diag/analyze_base_arm_dynamics.py /tmp/base-arm-C.jsonl --output /tmp/base-arm-C-summary.json
```

Compare source coverage and simulation-time spans first. Then compare robot/chassis pose displacement, yaw change, wheel and arm joint velocity statistics, and recorded base command components. Preserve the raw captures with the summaries.

## Interpretation limits

- The analyzer is descriptive only; it has no pass/fail threshold and does not establish causation.
- A temporal correlation between arm motion and chassis motion is not proof of reaction-force causality.
- A zero `cmd_vel` is not a mechanical brake or rigid wheel lock.
- A zero JointState `effort` value is not proof of zero physical torque or a locked wheel.
- Odometry is not interchangeable with Gazebo truth pose/twist. Report disagreements explicitly.
- Missing chassis-link or contact telemetry remains missing. Do not replace it with a model pose or fabricated estimate.
- Do not change production parameters based only on these summaries. Any physical parameter experiment must be separately designed as a controlled, single-variable test with safety review.

## Required handoff

For each group, retain: raw JSONL, analyzer JSON, exact git commit, world/configuration identity, initial pose, simulation duration, arm condition description, and confirmation that base command samples stayed zero. If any required source is absent or the initial conditions differ, mark the comparison inconclusive and repeat the capture after fixing the measurement/setup issue.
