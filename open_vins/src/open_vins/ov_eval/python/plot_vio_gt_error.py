#!/usr/bin/env python3
"""Plot OpenVINS VIO trajectory against groundtruth and report ATE errors."""

import argparse
import csv
import math
import os
from pathlib import Path

import numpy as np


def quat_xyzw_to_rot(q):
    q = np.asarray(q, dtype=float)
    n = np.linalg.norm(q)
    if n == 0:
        return np.eye(3)
    x, y, z, w = q / n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def yaw_from_rot(rot):
    return math.atan2(rot[1, 0], rot[0, 0])


def load_pose_file(path):
    """Load txt or EuRoC csv pose file into times, positions, quaternions xyzw."""
    path = Path(path)
    if not path.exists():
        raise FileNotFoundError(f"file does not exist: {path}")

    times = []
    positions = []
    quats = []

    with path.open("r", encoding="utf-8") as f:
        first_data_line = ""
        for line in f:
            stripped = line.strip()
            if stripped and not stripped.startswith("#"):
                first_data_line = stripped
                break

    delimiter = "," if "," in first_data_line else None
    with path.open("r", encoding="utf-8") as f:
        for row in f:
            row = row.strip()
            if not row or row.startswith("#"):
                continue
            parts = row.split(",") if delimiter == "," else row.split()
            if len(parts) < 8:
                continue
            values = [float(x) for x in parts[:8]]

            if delimiter == ",":
                # EuRoC csv format: time(ns), px, py, pz, qw, qx, qy, qz
                timestamp = values[0] * 1e-9 if values[0] > 1e12 else values[0]
                position = values[1:4]
                quat = [values[5], values[6], values[7], values[4]]
            else:
                # OpenVINS recorder txt format: time(s), tx, ty, tz, qx, qy, qz, qw
                timestamp = values[0]
                position = values[1:4]
                quat = values[4:8]

            times.append(timestamp)
            positions.append(position)
            quats.append(quat)

    if not times:
        raise RuntimeError(f"no poses loaded from: {path}")

    order = np.argsort(np.asarray(times))
    return np.asarray(times)[order], np.asarray(positions)[order], np.asarray(quats)[order]


def associate_by_time(est_t, gt_t, max_diff):
    est_idx = []
    gt_idx = []
    j = 0
    for i, t in enumerate(est_t):
        while j + 1 < len(gt_t) and gt_t[j + 1] < t:
            j += 1
        candidates = [j]
        if j + 1 < len(gt_t):
            candidates.append(j + 1)
        best = min(candidates, key=lambda k: abs(gt_t[k] - t))
        if abs(gt_t[best] - t) <= max_diff:
            est_idx.append(i)
            gt_idx.append(best)
    return np.asarray(est_idx, dtype=int), np.asarray(gt_idx, dtype=int)


def align_positions(est_p, gt_p, mode):
    if mode == "none":
        return np.eye(3), np.zeros(3), 1.0

    if mode == "posyaw":
        delta = gt_p[0] - est_p[0]
        yaw = math.atan2(gt_p[-1, 1] - gt_p[0, 1], gt_p[-1, 0] - gt_p[0, 0]) - math.atan2(
            est_p[-1, 1] - est_p[0, 1], est_p[-1, 0] - est_p[0, 0]
        )
        c, s = math.cos(yaw), math.sin(yaw)
        rot = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        trans = gt_p[0] - rot @ est_p[0]
        # Keep z initialized from the first pair as OpenVINS visualizer does.
        trans[2] = delta[2]
        return rot, trans, 1.0

    use_scale = mode == "sim3"
    src_mean = est_p.mean(axis=0)
    dst_mean = gt_p.mean(axis=0)
    src_centered = est_p - src_mean
    dst_centered = gt_p - dst_mean

    cov = src_centered.T @ dst_centered / len(est_p)
    u, singular_values, vt = np.linalg.svd(cov)
    d = np.eye(3)
    if np.linalg.det(vt.T @ u.T) < 0:
        d[-1, -1] = -1
    rot = vt.T @ d @ u.T
    scale = 1.0
    if use_scale:
        variance = np.sum(src_centered * src_centered) / len(est_p)
        scale = np.trace(np.diag(singular_values) @ d) / variance
    trans = dst_mean - scale * rot @ src_mean
    return rot, trans, scale


def rotation_angle_error_deg(est_q, gt_q, align_rot):
    errors = []
    for qe, qg in zip(est_q, gt_q):
        re = align_rot @ quat_xyzw_to_rot(qe)
        rg = quat_xyzw_to_rot(qg)
        cos_angle = (np.trace(rg.T @ re) - 1.0) * 0.5
        cos_angle = float(np.clip(cos_angle, -1.0, 1.0))
        errors.append(math.degrees(math.acos(cos_angle)))
    return np.asarray(errors)


def summarize(name, values, unit):
    return (
        f"{name}: rmse={math.sqrt(np.mean(values ** 2)):.4f}{unit}, "
        f"mean={np.mean(values):.4f}{unit}, median={np.median(values):.4f}{unit}, "
        f"max={np.max(values):.4f}{unit}"
    )


def save_error_csv(path, times, est_p, gt_p, pos_err, rot_err):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "timestamp",
                "est_x_aligned",
                "est_y_aligned",
                "est_z_aligned",
                "gt_x",
                "gt_y",
                "gt_z",
                "position_error_m",
                "rotation_error_deg",
            ]
        )
        for row in zip(times, est_p, gt_p, pos_err, rot_err):
            t, pe, pg, ep, er = row
            writer.writerow([f"{t:.9f}", *[f"{x:.9f}" for x in pe], *[f"{x:.9f}" for x in pg], f"{ep:.9f}", f"{er:.9f}"])


def prepare_matplotlib(show):
    import matplotlib

    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D  # noqa: F401

    return plt


def save_path_plot(path, est_p, gt_p, title, show):
    plt = prepare_matplotlib(show)

    fig = plt.figure(figsize=(13, 6))
    fig.suptitle(title)

    ax_xy = fig.add_subplot(1, 2, 1)
    ax_xy.plot(gt_p[:, 0], gt_p[:, 1], label="groundtruth", linewidth=2)
    ax_xy.plot(est_p[:, 0], est_p[:, 1], label="vio aligned", linewidth=1.5)
    ax_xy.set_title("XY trajectory")
    ax_xy.set_xlabel("x [m]")
    ax_xy.set_ylabel("y [m]")
    ax_xy.axis("equal")
    ax_xy.grid(True)
    ax_xy.legend()

    ax_3d = fig.add_subplot(1, 2, 2, projection="3d")
    ax_3d.plot(gt_p[:, 0], gt_p[:, 1], gt_p[:, 2], label="groundtruth", linewidth=2)
    ax_3d.plot(est_p[:, 0], est_p[:, 1], est_p[:, 2], label="vio aligned", linewidth=1.5)
    ax_3d.set_title("3D trajectory")
    ax_3d.set_xlabel("x [m]")
    ax_3d.set_ylabel("y [m]")
    ax_3d.set_zlabel("z [m]")
    ax_3d.legend()

    fig.tight_layout()
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=160)
    if show:
        plt.show()
    plt.close(fig)


def save_error_plot(path, times, pos_err, rot_err, title, show):
    plt = prepare_matplotlib(show)

    rel_t = times - times[0]
    fig = plt.figure(figsize=(13, 6))
    fig.suptitle(title)

    ax_pos = fig.add_subplot(1, 2, 1)
    ax_pos.plot(rel_t, pos_err, color="tab:red")
    ax_pos.set_title("Position error")
    ax_pos.set_xlabel("time [s]")
    ax_pos.set_ylabel("ATE [m]")
    ax_pos.grid(True)

    ax_rot = fig.add_subplot(1, 2, 2)
    ax_rot.plot(rel_t, rot_err, color="tab:purple")
    ax_rot.set_title("Rotation error")
    ax_rot.set_xlabel("time [s]")
    ax_rot.set_ylabel("angle [deg]")
    ax_rot.grid(True)

    fig.tight_layout()
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=160)
    if show:
        plt.show()
    plt.close(fig)


def make_plot(path, times, est_p, gt_p, pos_err, rot_err, title, show):
    plt = prepare_matplotlib(show)

    rel_t = times - times[0]
    fig = plt.figure(figsize=(14, 9))
    fig.suptitle(title)

    ax_xy = fig.add_subplot(2, 2, 1)
    ax_xy.plot(gt_p[:, 0], gt_p[:, 1], label="groundtruth", linewidth=2)
    ax_xy.plot(est_p[:, 0], est_p[:, 1], label="vio aligned", linewidth=1.5)
    ax_xy.set_title("XY trajectory")
    ax_xy.set_xlabel("x [m]")
    ax_xy.set_ylabel("y [m]")
    ax_xy.axis("equal")
    ax_xy.grid(True)
    ax_xy.legend()

    ax_z = fig.add_subplot(2, 2, 2)
    ax_z.plot(rel_t, gt_p[:, 2], label="groundtruth", linewidth=2)
    ax_z.plot(rel_t, est_p[:, 2], label="vio aligned", linewidth=1.5)
    ax_z.set_title("Z over time")
    ax_z.set_xlabel("time [s]")
    ax_z.set_ylabel("z [m]")
    ax_z.grid(True)
    ax_z.legend()

    ax_pos = fig.add_subplot(2, 2, 3)
    ax_pos.plot(rel_t, pos_err, color="tab:red")
    ax_pos.set_title("Position error")
    ax_pos.set_xlabel("time [s]")
    ax_pos.set_ylabel("ATE [m]")
    ax_pos.grid(True)

    ax_rot = fig.add_subplot(2, 2, 4)
    ax_rot.plot(rel_t, rot_err, color="tab:purple")
    ax_rot.set_title("Rotation error")
    ax_rot.set_xlabel("time [s]")
    ax_rot.set_ylabel("angle [deg]")
    ax_rot.grid(True)

    fig.tight_layout()
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=160)
    if show:
        plt.show()
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description="Plot VIO trajectory vs groundtruth and compute errors.")
    parser.add_argument("--est", default="/home/cyc/open_vins/traj_estimate.txt", help="OpenVINS estimated trajectory txt")
    parser.add_argument(
        "--gt",
        default="/home/cyc/open_vins/src/open_vins/ov_data/euroc_mav/V1_01_easy.txt",
        help="groundtruth txt or EuRoC csv",
    )
    parser.add_argument("--align", choices=["se3", "sim3", "posyaw", "none"], default="se3", help="alignment mode")
    parser.add_argument("--max-diff", type=float, default=0.02, help="maximum timestamp association difference in seconds")
    parser.add_argument("--plot", default="/home/cyc/open_vins/vio_gt_trajectory_error.png", help="output plot image path")
    parser.add_argument("--path-plot", default="/home/cyc/open_vins/vio_gt_path.png", help="output trajectory-only image path")
    parser.add_argument("--error-plot", default="/home/cyc/open_vins/vio_gt_error.png", help="output error-only image path")
    parser.add_argument("--csv", default="/home/cyc/open_vins/vio_gt_errors.csv", help="output associated error csv path")
    parser.add_argument("--show", action="store_true", help="display plot window")
    args = parser.parse_args()

    est_t, est_p, est_q = load_pose_file(args.est)
    gt_t, gt_p, gt_q = load_pose_file(args.gt)

    est_idx, gt_idx = associate_by_time(est_t, gt_t, args.max_diff)
    if len(est_idx) < 3:
        raise RuntimeError(
            f"only {len(est_idx)} matched poses; increase --max-diff or check that estimate and GT timestamps overlap"
        )

    est_t_m = est_t[est_idx]
    est_p_m = est_p[est_idx]
    est_q_m = est_q[est_idx]
    gt_p_m = gt_p[gt_idx]
    gt_q_m = gt_q[gt_idx]

    align_rot, align_trans, align_scale = align_positions(est_p_m, gt_p_m, args.align)
    est_p_aligned = align_scale * (align_rot @ est_p_m.T).T + align_trans

    pos_err = np.linalg.norm(est_p_aligned - gt_p_m, axis=1)
    rot_err = rotation_angle_error_deg(est_q_m, gt_q_m, align_rot)

    print(f"estimate file: {os.path.abspath(args.est)}")
    print(f"groundtruth file: {os.path.abspath(args.gt)}")
    print(f"matched poses: {len(est_idx)} / estimate={len(est_t)}, groundtruth={len(gt_t)}")
    print(f"alignment: {args.align}, scale={align_scale:.8f}, max_diff={args.max_diff:.3f}s")
    print(summarize("position", pos_err, "m"))
    print(summarize("rotation", rot_err, "deg"))

    save_error_csv(args.csv, est_t_m, est_p_aligned, gt_p_m, pos_err, rot_err)
    save_path_plot(args.path_plot, est_p_aligned, gt_p_m, "VIO vs Groundtruth Path", args.show)
    save_error_plot(args.error_plot, est_t_m, pos_err, rot_err, "VIO Error", args.show)
    make_plot(args.plot, est_t_m, est_p_aligned, gt_p_m, pos_err, rot_err, "VIO vs Groundtruth", args.show)
    print(f"saved path plot:  {os.path.abspath(args.path_plot)}")
    print(f"saved error plot: {os.path.abspath(args.error_plot)}")
    print(f"saved plot: {os.path.abspath(args.plot)}")
    print(f"saved csv:  {os.path.abspath(args.csv)}")


if __name__ == "__main__":
    main()
