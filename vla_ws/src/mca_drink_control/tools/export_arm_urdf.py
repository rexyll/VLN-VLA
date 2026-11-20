#!/usr/bin/env python3
"""Generate serial URDF models from vla_follow.arm_kin, the existing kinematic source of truth."""
import argparse
import math
import pathlib
import sys

import numpy as np


def fmt(values):
    return " ".join(f"{float(v):.12g}" for v in values)


def build_urdf(kin, side):
    links = ["base"] + [f"{side}_L{i}" for i in range(7)] + [f"{side}_palm"]
    out = [f'<robot name="drink_arm_{side}">']
    out.extend(f'  <link name="{name}"/>' for name in links)
    parent = "base"
    for i, (joint_name, offset, axis, limits) in enumerate(kin.JOINTS):
        child = f"{side}_L{i}"
        xyz = kin.SHOULDER[side] if i == 0 else offset
        lo, hi = limits
        out.extend([
            f'  <joint name="{side}_{joint_name}_joint" type="revolute">',
            f'    <parent link="{parent}"/>',
            f'    <child link="{child}"/>',
            f'    <origin xyz="{fmt(xyz)}" rpy="0 0 0"/>',
            f'    <axis xyz="{fmt(axis)}"/>',
            f'    <limit lower="{lo:.12g}" upper="{hi:.12g}" effort="400" velocity="8"/>',
            '  </joint>',
        ])
        parent = child
    out.extend([
        '  <joint name="' + side + '_palm_joint" type="fixed">',
        f'    <parent link="{parent}"/>',
        f'    <child link="{side}_palm"/>',
        f'    <origin xyz="0 0 {-kin.PALM_OFF:.12g}" rpy="0 0 0"/>',
        '  </joint>',
        '</robot>',
        '',
    ])
    return "\n".join(out)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", required=True)
    args = parser.parse_args()
    sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / "vla_follow"))
    from vla_follow import arm_kin as kin

    out_dir = pathlib.Path(args.output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    q_samples = [
        np.asarray(kin.Q_REST, dtype=float),
        np.zeros(kin.NJ, dtype=float),
        np.asarray([0.1, -0.4, 0.2, -0.8, 0.3, 0.2, -0.1], dtype=float),
    ]
    for side in ("R", "L"):
        (out_dir / f"drink_arm_{side}.urdf").write_text(
            build_urdf(kin, side), encoding="utf-8"
        )
        rows = []
        for q in q_samples:
            transform = kin.fk(side, q)
            rows.append(np.concatenate([q, transform.reshape(-1)]))
        np.savetxt(out_dir / f"drink_arm_{side}_fk_reference.txt",
                   np.asarray(rows), fmt="%.17g")


if __name__ == "__main__":
    main()
