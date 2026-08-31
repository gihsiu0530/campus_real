#!/usr/bin/env python3
"""Generate OpenVINS Hanxiang camera-IMU extrinsic candidate configs.

This enumerates discrete camera mounting hypotheses:
- horizontal optical-axis heading: forward, left, back, right
- vertical optical-axis component: down or up, at 45 degrees
- image roll around the optical axis: 0, 90, 180, 270 degrees

The generated configs are useful for short bag tests when camera-IMU extrinsics
were not calibrated.
"""

import argparse
import math
import shutil
from pathlib import Path


HEADINGS = {
    "front": (1.0, 0.0, 0.0),
    "left": (0.0, 1.0, 0.0),
    "back": (-1.0, 0.0, 0.0),
    "right": (0.0, -1.0, 0.0),
}

UP = (0.0, 0.0, 1.0)


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def norm(v):
    return math.sqrt(dot(v, v))


def normalize(v):
    n = norm(v)
    if n <= 0.0:
        raise ValueError("zero vector")
    return tuple(x / n for x in v)


def neg(v):
    return tuple(-x for x in v)


def add(a, b):
    return tuple(x + y for x, y in zip(a, b))


def scale(s, v):
    return tuple(s * x for x in v)


def roll_axes(x0, y0, z0, roll_deg):
    if roll_deg == 0:
        return x0, y0, z0
    if roll_deg == 90:
        return y0, neg(x0), z0
    if roll_deg == 180:
        return neg(x0), neg(y0), z0
    if roll_deg == 270:
        return neg(y0), x0, z0
    raise ValueError(f"unsupported roll: {roll_deg}")


def make_rotation(heading, vertical, roll_deg):
    h = HEADINGS[heading]
    vertical_vec = neg(UP) if vertical == "down" else UP
    z_cam_in_imu = normalize(add(h, vertical_vec))

    # Baseline landscape orientation:
    # image right is horizontal vehicle-right relative to the optical-axis heading.
    x0 = normalize(cross(h, UP))
    y0 = normalize(cross(z_cam_in_imu, x0))
    x_cam, y_cam, z_cam = roll_axes(x0, y0, z_cam_in_imu, roll_deg)

    # Columns are camera x/y/z axes expressed in IMU coordinates.
    return [
        [x_cam[0], y_cam[0], z_cam[0]],
        [x_cam[1], y_cam[1], z_cam[1]],
        [x_cam[2], y_cam[2], z_cam[2]],
    ]


def format_matrix(rotation, translation):
    rows = []
    for row, t in zip(rotation, translation):
        rows.append("    - [{:.12f}, {:.12f}, {:.12f}, {:.12f}]".format(row[0], row[1], row[2], t))
    rows.append("    - [0.0, 0.0, 0.0, 1.0]")
    return "\n".join(rows)


def replace_transform(text, matrix_text, candidate_name):
    lines = text.splitlines()
    out = []
    i = 0
    while i < len(lines):
        line = lines[i]
        if "candidate " in line:
            out.append(f"#   * generated candidate: {candidate_name}")
            i += 1
            continue
        if line.strip() == "T_imu_cam:":
            out.append(line)
            out.extend(matrix_text.splitlines())
            i += 5
            continue
        out.append(line)
        i += 1
    return "\n".join(out) + "\n"


def generate(base_dir, output_parent, prefix, only_down):
    base_dir = Path(base_dir)
    output_parent = Path(output_parent)
    estimator = base_dir / "estimator_config.yaml"
    imu = base_dir / "kalibr_imu_chain.yaml"
    imucam = base_dir / "kalibr_imucam_chain.yaml"

    if not estimator.exists() or not imu.exists() or not imucam.exists():
        raise RuntimeError(f"Base config directory is incomplete: {base_dir}")

    imucam_text = imucam.read_text()
    verticals = ["down"] if only_down else ["down", "up"]
    generated = []

    for heading in HEADINGS:
        for vertical in verticals:
            for roll in (0, 90, 180, 270):
                name = f"{prefix}_{heading}_{vertical}_roll{roll:03d}"
                out_dir = output_parent / name
                out_dir.mkdir(parents=True, exist_ok=True)
                shutil.copy2(estimator, out_dir / "estimator_config.yaml")
                shutil.copy2(imu, out_dir / "kalibr_imu_chain.yaml")

                rotation = make_rotation(heading, vertical, roll)
                matrix_text = format_matrix(rotation, (0.0, 0.0, -0.23))
                candidate_text = replace_transform(imucam_text, matrix_text, name)
                (out_dir / "kalibr_imucam_chain.yaml").write_text(candidate_text)
                generated.append(name)

    for name in generated:
        print(name)
    print(f"Generated {len(generated)} configs under {output_parent}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--base-dir",
        default="src/open_vins/config/hanxiang_drone",
        help="Base config directory to copy",
    )
    parser.add_argument(
        "--output-parent",
        default="src/open_vins/config",
        help="Parent directory for generated configs",
    )
    parser.add_argument("--prefix", default="hanxiang_drone_cand", help="Generated config folder prefix")
    parser.add_argument("--only-down", action="store_true", help="Only generate downward-looking candidates")
    args = parser.parse_args()
    generate(args.base_dir, args.output_parent, args.prefix, args.only_down)


if __name__ == "__main__":
    main()
