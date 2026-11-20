# mca_drink_control

Selective, pinned integration of reusable MotionControlAlgorithms components for
VLN-VLA DrinkTask.

## Upstream source

- Repository: https://github.com/rexyll/MotionControlAlgorithms
- Pinned commit: `c11091bd2bb3ce001eb09f2664c1ff5076f8a73b`
- License: Apache-2.0 (see `third_party/MotionControlAlgorithms/LICENSE` and `NOTICE`)
- Import policy: selected header-only algorithm dependency snapshot; no full repository merge.

## Current scope

The package contains a dependency-light C++17 core wrapper for:

- Damped least-squares IK using MCA RobotModel and Pose3D
- Per-joint S-curve trajectory sampling and synchronized trajectory helpers
- Differential-drive body-twist to wheel-rate mapping

It also builds an opt-in ROS 2 Jazzy service:

- Service: `/mca_drink/solve_ik`
- Type: `mca_drink_control/srv/SolveIk`
- Server executable: `ros2 run mca_drink_control mca_ik_server`
- Arm URDF models are generated from the authoritative `vla_follow.arm_kin` model.
- The service accepts a target expressed in the robot base frame and a seven-joint seed; its response includes the solution, iteration count, position/orientation residuals and status message.
- The DrinkTask caller may retain the established IK path when the service is unavailable or fails. MCA DLS remains a candidate refinement path behind existing safety gates; it does not bypass human-posture checks, joint limits, collision/swept-path checks or measured-arrival validation.

## Execution boundary

- This package does not publish any of the 39 `/drink/cmd/*` joint-command topics.
- Python `DrinkTask/CmdBus` remains the sole publisher of those topics.
- The ROS service does not provide a physical chassis lock and does not replace Nav2.
- S-curve trajectory and differential-drive components are available in the C++ core, but are not yet a wholesale replacement for the existing production trajectory and base-motion controllers.
- Cartesian impedance/admittance, task-space hierarchy and CBF are not connected to the production execution loop.

## Build and test

From the ROS 2 Jazzy environment:

```bash
source /opt/ros/jazzy/setup.bash
cd vla_ws
colcon build --symlink-install --packages-select mca_drink_control
source install/setup.bash
colcon test --packages-select mca_drink_control --event-handlers console_direct+
colcon test-result --verbose
ros2 interface show mca_drink_control/srv/SolveIk
```

When the demo is launched by `tools/start_drink_demo.sh`, it starts
`mca_ik_server` and writes its log to `vla_ws/logs/mca_ik.log`.

See `MCA_UPSTREAM.lock` for the exact imported file list and source provenance.
