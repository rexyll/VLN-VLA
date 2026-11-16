# mca_drink_control

Selective, pinned integration of reusable MotionControlAlgorithms components for
VLN-VLA DrinkTask.

## Upstream source

- Repository: https://github.com/rexyll/MotionControlAlgorithms
- Pinned commit: `c11091bd2bb3ce001eb09f2664c1ff5076f8a73b`
- License: Apache-2.0 (see `third_party/MotionControlAlgorithms/LICENSE` and `NOTICE`)
- Import policy: selected header-only algorithm dependency snapshot; no full repository merge.

## Current scope

The initial package exposes a dependency-light C++17 core wrapper for:

- Damped least-squares IK using MCA `RobotModel` and `Pose3D`
- Per-joint S-curve trajectory sampling
- Differential-drive body-twist to wheel-rate mapping

It includes C++ unit tests for a reachable serial chain, trajectory limits and
differential-drive kinematics.

## Explicit non-goals at this stage

- No ROS 2 node is enabled yet.
- No topic in `/drink/cmd/*` is published by this package.
- No production solver or trajectory has been switched from Python/KDL.
- No physical chassis lock is provided by these algorithm headers.
- Cartesian impedance/admittance, task-space hierarchy and CBF are not wired into
  the production control loop.

The Python `DrinkTask/CmdBus` remains the sole publisher of the 39 joint-command
topics until a separately tested ROS adapter is approved.

## Build and test

From the ROS 2 Jazzy environment:

```bash
source /opt/ros/jazzy/setup.bash
cd vla_ws
colcon build --symlink-install --packages-select mca_drink_control
source install/setup.bash
colcon test --packages-select mca_drink_control --event-handlers console_direct+
colcon test-result --verbose
```

See `MCA_UPSTREAM.lock` for the exact imported file list and source provenance.
