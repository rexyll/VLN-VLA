#!/usr/bin/env python3
"""Exercise the installed MCA DLS ROS 2 service against both generated arm models."""
import math
import time

def matrix_quaternion(rotation):
    import numpy as np
    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        return ((rotation[2, 1] - rotation[1, 2]) / scale,
                (rotation[0, 2] - rotation[2, 0]) / scale,
                (rotation[1, 0] - rotation[0, 1]) / scale,
                0.25 * scale)
    if rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
        scale = math.sqrt(1.0 + rotation[0, 0] -
                          rotation[1, 1] - rotation[2, 2]) * 2.0
        return (0.25 * scale,
                (rotation[0, 1] + rotation[1, 0]) / scale,
                (rotation[0, 2] + rotation[2, 0]) / scale,
                (rotation[2, 1] - rotation[1, 2]) / scale)
    if rotation[1, 1] > rotation[2, 2]:
        scale = math.sqrt(1.0 + rotation[1, 1] -
                          rotation[0, 0] - rotation[2, 2]) * 2.0
        return ((rotation[0, 1] + rotation[1, 0]) / scale,
                0.25 * scale,
                (rotation[1, 2] + rotation[2, 1]) / scale,
                (rotation[0, 2] - rotation[2, 0]) / scale)
    scale = math.sqrt(1.0 + rotation[2, 2] -
                      rotation[0, 0] - rotation[1, 1]) * 2.0
    return ((rotation[0, 2] + rotation[2, 0]) / scale,
            (rotation[1, 2] + rotation[2, 1]) / scale,
            0.25 * scale,
            (rotation[1, 0] - rotation[0, 1]) / scale)


def main():
    import numpy as np
    import rclpy
    from mca_drink_control.srv import SolveIk
    from rclpy.node import Node
    from vla_follow import arm_kin as K

    rclpy.init()
    node = Node("mca_ik_service_smoke_test")
    client = node.create_client(SolveIk, "/mca_drink/solve_ik")
    try:
        if not client.wait_for_service(timeout_sec=15.0):
            raise RuntimeError("MCA DLS service did not become ready")
        delta = np.asarray([0.02, -0.02, 0.01, -0.02, 0.01, 0.02, -0.01])
        seed = np.asarray(K.Q_REST, dtype=float)
        for side in ("R", "L"):
            target_q = np.clip(seed + delta, K.LIM_LO, K.LIM_HI)
            target = K.fk(side, target_q)
            request = SolveIk.Request()
            request.side = side
            request.target.position.x = float(target[0, 3])
            request.target.position.y = float(target[1, 3])
            request.target.position.z = float(target[2, 3])
            qx, qy, qz, qw = matrix_quaternion(target[:3, :3])
            request.target.orientation.x = qx
            request.target.orientation.y = qy
            request.target.orientation.z = qz
            request.target.orientation.w = qw
            request.seed.name = [
                '%s_%s_joint' % (side, joint[0]) for joint in K.JOINTS]
            request.seed.position = [float(v) for v in seed]
            request.position_tolerance_m = 1e-4
            request.orientation_tolerance_rad = 1e-4
            request.max_iterations = 300
            future = client.call_async(request)
            deadline = time.monotonic() + 10.0
            while not future.done() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.02)
            if not future.done():
                raise RuntimeError(f"{side}: service call timed out")
            response = future.result()
            if response is None or not response.success:
                raise RuntimeError(
                    f"{side}: MCA DLS failed: "
                    f"{getattr(response, 'message', 'no response')}")
            expected_names = [
                '%s_%s_joint' % (side, joint[0]) for joint in K.JOINTS]
            if list(response.solution.name) != expected_names:
                raise RuntimeError(f"{side}: returned joint names/order mismatch")
            solved = np.asarray(response.solution.position, dtype=float)
            if solved.shape != (K.NJ,) or not np.all(np.isfinite(solved)):
                raise RuntimeError(f"{side}: invalid returned joint vector")
            actual = K.fk(side, solved)
            pos_mm = float(np.linalg.norm(actual[:3, 3] - target[:3, 3]) * 1000.0)
            rot = actual[:3, :3] @ target[:3, :3].T
            angle = math.degrees(math.acos(max(-1.0, min(
                1.0, (float(np.trace(rot)) - 1.0) / 2.0))))
            if pos_mm > 0.2 or angle > 0.02:
                raise RuntimeError(
                    f"{side}: FK residual too large: {pos_mm:.4f}mm/{angle:.5f}deg")
            print(f"MCA_DLS_SERVICE_PASS side={side} "
                  f"position_mm={pos_mm:.5f} axis_deg={angle:.6f} "
                  f"iterations={response.iterations}")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
