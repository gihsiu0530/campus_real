#!/usr/bin/env python3
"""Extract odometry from a bag as zero-start GT and compare it with VIO CSV."""

import argparse
import csv
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import numpy as np
import rosbag


def quat_xyzw_to_rot(q):
    q = np.asarray(q, dtype=float)
    n = np.linalg.norm(q)
    if n == 0.0:
        return np.eye(3)
    x, y, z, w = q / n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def rot_to_quat_xyzw(rot):
    tr = np.trace(rot)
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * s
        qx = (rot[2, 1] - rot[1, 2]) / s
        qy = (rot[0, 2] - rot[2, 0]) / s
        qz = (rot[1, 0] - rot[0, 1]) / s
    else:
        idx = int(np.argmax(np.diag(rot)))
        if idx == 0:
            s = math.sqrt(1.0 + rot[0, 0] - rot[1, 1] - rot[2, 2]) * 2.0
            qw = (rot[2, 1] - rot[1, 2]) / s
            qx = 0.25 * s
            qy = (rot[0, 1] + rot[1, 0]) / s
            qz = (rot[0, 2] + rot[2, 0]) / s
        elif idx == 1:
            s = math.sqrt(1.0 + rot[1, 1] - rot[0, 0] - rot[2, 2]) * 2.0
            qw = (rot[0, 2] - rot[2, 0]) / s
            qx = (rot[0, 1] + rot[1, 0]) / s
            qy = 0.25 * s
            qz = (rot[1, 2] + rot[2, 1]) / s
        else:
            s = math.sqrt(1.0 + rot[2, 2] - rot[0, 0] - rot[1, 1]) * 2.0
            qw = (rot[1, 0] - rot[0, 1]) / s
            qx = (rot[0, 2] + rot[2, 0]) / s
            qy = (rot[1, 2] + rot[2, 1]) / s
            qz = 0.25 * s
    q = np.array([qx, qy, qz, qw], dtype=float)
    return q / np.linalg.norm(q)


def pose_from_msg(msg):
    if hasattr(msg, "pose") and hasattr(msg.pose, "pose"):
        pose = msg.pose.pose
    elif hasattr(msg, "pose"):
        pose = msg.pose
    else:
        raise TypeError("topic message must be nav_msgs/Odometry or geometry_msgs/PoseStamped")
    p = np.array([pose.position.x, pose.position.y, pose.position.z], dtype=float)
    q = np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w], dtype=float)
    return p, q


def msg_time(msg, bag_time):
    stamp = getattr(getattr(msg, "header", None), "stamp", None)
    if stamp is not None and stamp.to_sec() > 0.0:
        return stamp.to_sec()
    return bag_time.to_sec()


def extract_zero_start_odom(bag_path, topic):
    times = []
    positions = []
    quats = []
    with rosbag.Bag(str(bag_path), "r") as bag:
        for _, msg, bag_time in bag.read_messages(topics=[topic]):
            p, q = pose_from_msg(msg)
            times.append(msg_time(msg, bag_time))
            positions.append(p)
            quats.append(q)

    if not times:
        raise RuntimeError(f"no messages found for topic {topic} in {bag_path}")

    times = np.asarray(times, dtype=float)
    positions = np.asarray(positions, dtype=float)
    quats = np.asarray(quats, dtype=float)
    order = np.argsort(times)
    times = times[order]
    positions = positions[order]
    quats = quats[order]

    p0 = positions[0]
    r0_inv = quat_xyzw_to_rot(quats[0]).T

    out_positions = []
    out_quats = []
    for p, q in zip(positions, quats):
        out_positions.append(r0_inv @ (p - p0))
        out_quats.append(rot_to_quat_xyzw(r0_inv @ quat_xyzw_to_rot(q)))
    return times, np.asarray(out_positions), np.asarray(out_quats)


def save_pose_csv(path, times, positions, quats):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["timestamp(ns)", "tx", "ty", "tz", "qw", "qx", "qy", "qz"])
        for t, p, q in zip(times, positions, quats):
            writer.writerow([f"{t * 1e9:.0f}", f"{p[0]:.9f}", f"{p[1]:.9f}", f"{p[2]:.9f}", f"{q[3]:.9f}", f"{q[0]:.9f}", f"{q[1]:.9f}", f"{q[2]:.9f}"])


def load_pose_csv(path):
    times = []
    positions = []
    quats = []
    with Path(path).open("r", encoding="utf-8") as f:
        reader = csv.reader(f)
        for row in reader:
            if not row or row[0].startswith("#") or row[0].lower().startswith("timestamp"):
                continue
            values = [float(v) for v in row[:8]]
            t = values[0] * 1e-9 if values[0] > 1e12 else values[0]
            times.append(t)
            positions.append(values[1:4])
            quats.append([values[5], values[6], values[7], values[4]])
    if not times:
        raise RuntimeError(f"no poses loaded from {path}")
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
        est_xy = est_p[:, :2] - est_p[0, :2]
        gt_xy = gt_p[:, :2] - gt_p[0, :2]
        cross = np.sum(est_xy[:, 0] * gt_xy[:, 1] - est_xy[:, 1] * gt_xy[:, 0])
        dot = np.sum(est_xy[:, 0] * gt_xy[:, 0] + est_xy[:, 1] * gt_xy[:, 1])
        yaw = math.atan2(cross, dot)
        c, s = math.cos(yaw), math.sin(yaw)
        rot = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        return rot, gt_p[0] - rot @ est_p[0], 1.0

    src_mean = est_p.mean(axis=0)
    dst_mean = gt_p.mean(axis=0)
    src = est_p - src_mean
    dst = gt_p - dst_mean
    cov = src.T @ dst / len(est_p)
    u, singular_values, vt = np.linalg.svd(cov)
    d = np.eye(3)
    if np.linalg.det(vt.T @ u.T) < 0:
        d[-1, -1] = -1
    rot = vt.T @ d @ u.T
    scale = 1.0
    if mode == "sim3":
        scale = np.trace(np.diag(singular_values) @ d) / (np.sum(src * src) / len(est_p))
    return rot, dst_mean - scale * rot @ src_mean, scale


def rotation_errors_deg(est_q, gt_q, align_rot):
    errors = []
    for qe, qg in zip(est_q, gt_q):
        re = align_rot @ quat_xyzw_to_rot(qe)
        rg = quat_xyzw_to_rot(qg)
        cos_angle = (np.trace(rg.T @ re) - 1.0) * 0.5
        errors.append(math.degrees(math.acos(float(np.clip(cos_angle, -1.0, 1.0)))))
    return np.asarray(errors)


def save_error_csv(path, times, est_p_raw, est_p_compared, gt_p, pos_diff, pos_err, xy_err, rot_err):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "timestamp(s)",
                "est_x_raw",
                "est_y_raw",
                "est_z_raw",
                "est_x_compared",
                "est_y_compared",
                "est_z_compared",
                "gt_x",
                "gt_y",
                "gt_z",
                "error_x_m",
                "error_y_m",
                "error_z_m",
                "plot_dx_m",
                "plot_dy_m",
                "plot_xy_error_m",
                "xy_error_m",
                "position_error_m",
                "rotation_error_deg",
            ]
        )
        for t, pe_raw, pe_cmp, pg, dp, ep_xy, ep, er in zip(
            times, est_p_raw, est_p_compared, gt_p, pos_diff, xy_err, pos_err, rot_err
        ):
            writer.writerow(
                [
                    f"{t:.9f}",
                    *[f"{x:.9f}" for x in pe_raw],
                    *[f"{x:.9f}" for x in pe_cmp],
                    *[f"{x:.9f}" for x in pg],
                    *[f"{x:.9f}" for x in dp],
                    f"{dp[0]:.9f}",
                    f"{dp[1]:.9f}",
                    f"{ep_xy:.9f}",
                    f"{ep_xy:.9f}",
                    f"{ep:.9f}",
                    f"{er:.9f}",
                ]
            )


def save_xy_plot(path, est_p, gt_p, align_mode):
    import matplotlib.pyplot as plt

    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.plot(gt_p[:, 0], gt_p[:, 1], label="GT odom", linewidth=2.0)
    ax.plot(est_p[:, 0], est_p[:, 1], label=f"VIO ({align_mode})", linewidth=2.0)
    ax.scatter(gt_p[0, 0], gt_p[0, 1], marker="o", s=50, label="GT start")
    ax.scatter(est_p[0, 0], est_p[0, 1], marker="x", s=70, label="VIO start")
    ax.scatter(gt_p[-1, 0], gt_p[-1, 1], marker="s", s=50, label="GT end")
    ax.scatter(est_p[-1, 0], est_p[-1, 1], marker="+", s=90, label="VIO end")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("VIO vs GT XY")
    ax.axis("equal")
    ax.grid(True)
    ax.legend()
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def print_summary(name, values, unit):
    print(f"{name}: rmse={math.sqrt(np.mean(values ** 2)):.4f}{unit}, mean={np.mean(values):.4f}{unit}, median={np.median(values):.4f}{unit}, max={np.max(values):.4f}{unit}")


def main():
    parser = argparse.ArgumentParser(description="Extract bag odometry as zero-start GT and compare with OpenVINS CSV.")
    parser.add_argument("--bag", required=True, help="input rosbag path")
    parser.add_argument("--odom-topic", default="/odom", help="GT odometry topic in the bag")
    parser.add_argument("--vio", default="/home/cyc/open_vins/vio_estimate.csv", help="OpenVINS estimate CSV")
    parser.add_argument("--gt-csv", default="/home/cyc/open_vins/gt_odom_zero.csv", help="output zero-start GT CSV")
    parser.add_argument("--error-csv", default="/home/cyc/open_vins/vio_odom_errors.csv", help="output associated error CSV")
    parser.add_argument("--xy-plot", default="/home/cyc/open_vins/vio_gt_xy_overlay.png", help="output XY overlay plot")
    parser.add_argument("--no-plot", action="store_true", help="skip XY overlay plot")
    parser.add_argument("--max-diff", type=float, default=0.03, help="max timestamp difference for association [s]")
    parser.add_argument("--align", choices=["none", "posyaw", "se3", "sim3"], default="none", help="alignment before error calculation")
    args = parser.parse_args()

    gt_t, gt_p, gt_q = extract_zero_start_odom(Path(args.bag), args.odom_topic)
    save_pose_csv(args.gt_csv, gt_t, gt_p, gt_q)

    est_t, est_p, est_q = load_pose_csv(args.vio)
    est_idx, gt_idx = associate_by_time(est_t, gt_t, args.max_diff)
    if len(est_idx) < 3:
        raise RuntimeError(f"only {len(est_idx)} matched poses; check topic/timestamps or increase --max-diff")

    est_t_m = est_t[est_idx]
    est_p_m = est_p[est_idx]
    est_q_m = est_q[est_idx]
    gt_p_m = gt_p[gt_idx]
    gt_q_m = gt_q[gt_idx]

    align_rot, align_trans, align_scale = align_positions(est_p_m, gt_p_m, args.align)
    est_p_aligned = align_scale * (align_rot @ est_p_m.T).T + align_trans
    pos_diff = est_p_aligned - gt_p_m
    xy_err = np.linalg.norm(pos_diff[:, :2], axis=1)
    pos_err = np.linalg.norm(pos_diff, axis=1)
    rot_err = rotation_errors_deg(est_q_m, gt_q_m, align_rot)

    save_error_csv(args.error_csv, est_t_m, est_p_m, est_p_aligned, gt_p_m, pos_diff, pos_err, xy_err, rot_err)
    if not args.no_plot:
        save_xy_plot(args.xy_plot, est_p_aligned, gt_p_m, args.align)

    print(f"GT messages: {len(gt_t)} from {args.odom_topic}")
    print(f"matched poses: {len(est_idx)} / vio={len(est_t)}, gt={len(gt_t)}")
    print(f"alignment: {args.align}, scale={align_scale:.8f}, max_diff={args.max_diff:.3f}s")
    print_summary("plot XY position", xy_err, "m")
    print_summary("3D position", pos_err, "m")
    print_summary("rotation", rot_err, "deg")
    print(f"saved GT CSV:    {Path(args.gt_csv).resolve()}")
    print(f"saved error CSV: {Path(args.error_csv).resolve()}")
    if not args.no_plot:
        print(f"saved XY plot:   {Path(args.xy_plot).resolve()}")


if __name__ == "__main__":
    main()
