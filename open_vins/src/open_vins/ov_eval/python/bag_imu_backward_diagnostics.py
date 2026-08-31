#!/usr/bin/env python3
"""Diagnose why pure IMU dead reckoning starts moving backward.

The script compares IMU-integrated velocity/heading against GT odometry motion
heading. It is meant to distinguish data issues from integration/state issues.
"""

import argparse
import csv
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import numpy as np
import rosbag

import bag_imu_dead_reckoning as dr


def angle_wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def angle_wrap_array(a):
    return (a + np.pi) % (2.0 * np.pi) - np.pi


def quat_wxyz_to_rot(q):
    qw, qx, qy, qz = q
    return dr.quat_xyzw_to_rot(np.array([qx, qy, qz, qw], dtype=float))


def quat_yaw_wxyz(q):
    return dr.yaw_from_rot(quat_wxyz_to_rot(q))


def pose_yaw_xyzw(q):
    return dr.yaw_from_rot(dr.quat_xyzw_to_rot(q))


def read_odom_zeroed_with_yaw(bag_path, topic):
    times = []
    positions = []
    yaws = []
    with rosbag.Bag(str(bag_path), "r") as bag:
        for _, msg, bag_time in bag.read_messages(topics=[topic]):
            p, q = dr.pose_from_msg(msg)
            times.append(dr.msg_time(msg, bag_time))
            positions.append(p)
            yaws.append(pose_yaw_xyzw(q))
    if not times:
        raise RuntimeError(f"no GT messages found for topic {topic} in {bag_path}")

    order = np.argsort(np.asarray(times))
    times = np.asarray(times, dtype=float)[order]
    positions = np.asarray(positions, dtype=float)[order]
    yaws = np.unwrap(np.asarray(yaws, dtype=float)[order])

    p0 = positions[0]
    yaw0 = yaws[0]
    r0_inv = dr.yaw_rot(-yaw0)
    positions_zeroed = (r0_inv @ (positions - p0).T).T
    yaws_zeroed = angle_wrap_array(yaws - yaw0)
    return times, positions_zeroed, yaws_zeroed


def finite_difference_velocity(times, positions):
    times = np.asarray(times, dtype=float)
    positions = np.asarray(positions, dtype=float)
    velocities = np.zeros_like(positions)
    if len(times) < 2:
        return velocities
    dt = np.diff(times)
    valid = dt > 0.0
    segment_v = np.zeros((len(dt), positions.shape[1]), dtype=float)
    segment_v[valid] = (positions[1:][valid] - positions[:-1][valid]) / dt[valid, None]
    velocities[0] = segment_v[0]
    velocities[-1] = segment_v[-1]
    if len(times) > 2:
        velocities[1:-1] = 0.5 * (segment_v[:-1] + segment_v[1:])
    return velocities


def nearest_indices(query_t, source_t, max_diff):
    indices = []
    for t in query_t:
        right = int(np.searchsorted(source_t, t))
        candidates = []
        if right < len(source_t):
            candidates.append(right)
        if right > 0:
            candidates.append(right - 1)
        if not candidates:
            indices.append(-1)
            continue
        best = min(candidates, key=lambda i: abs(source_t[i] - t))
        indices.append(best if abs(source_t[best] - t) <= max_diff else -1)
    return np.asarray(indices, dtype=int)


def percentile_text(values, unit=""):
    if len(values) == 0:
        return "n/a"
    p = np.percentile(values, [0, 1, 50, 99, 100])
    return (
        f"min={p[0]:.6g}{unit}, p01={p[1]:.6g}{unit}, "
        f"median={p[2]:.6g}{unit}, p99={p[3]:.6g}{unit}, max={p[4]:.6g}{unit}"
    )


def first_true_segment(mask):
    idx = np.flatnonzero(mask)
    if len(idx) == 0:
        return None
    start = int(idx[0])
    end = start
    while end + 1 < len(mask) and mask[end + 1]:
        end += 1
    return start, end


def save_diagnostic_csv(path, rows):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = [
        "time_s",
        "rel_time_s",
        "imu_gt_time_diff_ms",
        "imu_x",
        "imu_y",
        "gt_x",
        "gt_y",
        "imu_vx",
        "imu_vy",
        "gt_vx",
        "gt_vy",
        "imu_speed",
        "gt_speed",
        "imu_vel_heading_deg",
        "gt_motion_heading_deg",
        "vel_heading_error_deg",
        "imu_yaw_deg",
        "gt_pose_yaw_deg",
        "yaw_error_deg",
        "ax_global",
        "ay_global",
        "az_global",
        "gyro_x",
        "gyro_y",
        "gyro_z",
        "accel_x",
        "accel_y",
        "accel_z",
        "backward_flag",
    ]
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def save_plots(output_dir, rel_t, heading_err, yaw_err, imu_speed, gt_speed, xy, backward_mask):
    import matplotlib.pyplot as plt

    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    axes[0].plot(rel_t, np.degrees(heading_err), label="IMU velocity vs GT motion")
    axes[0].axhline(120.0, color="r", linestyle="--", linewidth=1.0)
    axes[0].axhline(-120.0, color="r", linestyle="--", linewidth=1.0)
    axes[0].set_ylabel("heading error [deg]")
    axes[0].grid(True)
    axes[0].legend()

    axes[1].plot(rel_t, np.degrees(yaw_err), label="IMU yaw vs GT pose yaw")
    axes[1].set_ylabel("yaw error [deg]")
    axes[1].grid(True)
    axes[1].legend()

    axes[2].plot(rel_t, imu_speed, label="IMU speed")
    axes[2].plot(rel_t, gt_speed, label="GT speed")
    axes[2].set_ylabel("speed [m/s]")
    axes[2].set_xlabel("time from first IMU sample [s]")
    axes[2].grid(True)
    axes[2].legend()

    if np.any(backward_mask):
        for ax in axes:
            ax.scatter(rel_t[backward_mask], np.zeros(np.count_nonzero(backward_mask)), s=4, color="r", alpha=0.25)

    fig.tight_layout()
    fig.savefig(output_dir / "heading_speed_diagnostics.png", dpi=150)
    plt.close(fig)

    imu_p, gt_p = xy
    fig, ax = plt.subplots(figsize=(9, 9))
    ax.plot(gt_p[:, 0], gt_p[:, 1], label="GT odom", linewidth=2.0)
    ax.plot(imu_p[:, 0], imu_p[:, 1], label="IMU dead reckoning aligned", linewidth=2.0)
    if np.any(backward_mask):
        ax.scatter(imu_p[backward_mask, 0], imu_p[backward_mask, 1], s=10, color="r", label="backward candidate")
    ax.scatter(imu_p[0, 0], imu_p[0, 1], marker="x", s=70, label="IMU start")
    ax.scatter(imu_p[-1, 0], imu_p[-1, 1], marker="+", s=90, label="IMU end")
    ax.axis("equal")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.grid(True)
    ax.legend()
    fig.tight_layout()
    fig.savefig(output_dir / "xy_backward_candidates.png", dpi=150)
    plt.close(fig)


def write_summary(path, lines):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description="Diagnose backward motion in IMU dead reckoning.")
    parser.add_argument("--bag", required=True, help="input rosbag path")
    parser.add_argument("--imu-topic", default="/external_imu/data", help="IMU topic")
    parser.add_argument("--odom-topic", required=True, help="GT odom topic")
    parser.add_argument("--output-dir", default="/home/cyc/open_vins/imu_backward_diagnostics", help="diagnostic output directory")
    parser.add_argument("--static-duration", type=float, default=3.0, help="initial stationary window for bias/gravity init [s]")
    parser.add_argument("--gravity", type=float, default=9.81, help="gravity magnitude [m/s^2]")
    parser.add_argument("--accel-bias-mode", choices=["none", "gravity_magnitude"], default="gravity_magnitude")
    parser.add_argument("--align", choices=["none", "posyaw"], default="posyaw")
    parser.add_argument("--max-diff", type=float, default=0.03, help="max timestamp difference for GT association [s]")
    parser.add_argument("--gt-rate-only", action="store_true", help="diagnose only one nearest IMU state per GT sample")
    parser.add_argument("--backward-angle-deg", type=float, default=120.0, help="heading difference threshold for backward flag")
    parser.add_argument("--min-gt-speed", type=float, default=0.15, help="ignore near-stationary GT samples below this speed")
    parser.add_argument("--min-imu-speed", type=float, default=0.15, help="ignore near-stationary IMU estimates below this speed")
    args = parser.parse_args()

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    times, gyros, accels = dr.read_imu_bag(Path(args.bag), args.imu_topic)
    positions, velocities, quats, lin_accels, gyro_bias, accel_bias = dr.integrate_imu(
        times, gyros, accels, args.static_duration, args.gravity, args.accel_bias_mode
    )
    gt_t, gt_p, gt_yaw = read_odom_zeroed_with_yaw(Path(args.bag), args.odom_topic)

    if args.gt_rate_only:
        imu_for_gt = nearest_indices(gt_t, times, args.max_diff)
        valid = imu_for_gt >= 0
        imu_idx = imu_for_gt[valid]
        gt_idx = np.flatnonzero(valid)
    else:
        imu_idx, gt_idx = dr.associate_by_time(times, gt_t, args.max_diff)
    if len(imu_idx) < 3:
        raise RuntimeError(f"only {len(imu_idx)} matched poses; increase --max-diff or check timestamps")

    imu_p = positions[imu_idx]
    imu_v = velocities[imu_idx]
    imu_q = quats[imu_idx]
    imu_a_global = lin_accels[imu_idx]
    imu_g = gyros[imu_idx]
    imu_a_raw = accels[imu_idx]
    gt_pm = gt_p[gt_idx]
    gt_yawm = gt_yaw[gt_idx]
    match_times = times[imu_idx]
    gt_match_times = gt_t[gt_idx]

    rot, trans = dr.align_positions(imu_p, gt_pm, args.align)
    imu_p_aligned = (rot @ imu_p.T).T + trans
    imu_v_aligned = (rot @ imu_v.T).T
    imu_a_global_aligned = (rot @ imu_a_global.T).T
    align_yaw = dr.yaw_from_rot(rot)

    gt_v = finite_difference_velocity(gt_match_times, gt_pm)
    imu_speed = np.linalg.norm(imu_v_aligned[:, :2], axis=1)
    gt_speed = np.linalg.norm(gt_v[:, :2], axis=1)
    imu_vel_heading = np.arctan2(imu_v_aligned[:, 1], imu_v_aligned[:, 0])
    gt_motion_heading = np.arctan2(gt_v[:, 1], gt_v[:, 0])
    heading_err = angle_wrap_array(imu_vel_heading - gt_motion_heading)

    imu_yaw = np.asarray([angle_wrap(quat_yaw_wxyz(q) + align_yaw) for q in imu_q])
    yaw_err = angle_wrap_array(imu_yaw - gt_yawm)

    backward_mask = (
        (np.abs(np.degrees(heading_err)) >= args.backward_angle_deg)
        & (gt_speed >= args.min_gt_speed)
        & (imu_speed >= args.min_imu_speed)
    )
    first_segment = first_true_segment(backward_mask)

    rows = []
    for i in range(len(match_times)):
        rows.append(
            {
                "time_s": f"{match_times[i]:.9f}",
                "rel_time_s": f"{match_times[i] - times[0]:.9f}",
                "imu_gt_time_diff_ms": f"{(match_times[i] - gt_match_times[i]) * 1000.0:.6f}",
                "imu_x": f"{imu_p_aligned[i, 0]:.9f}",
                "imu_y": f"{imu_p_aligned[i, 1]:.9f}",
                "gt_x": f"{gt_pm[i, 0]:.9f}",
                "gt_y": f"{gt_pm[i, 1]:.9f}",
                "imu_vx": f"{imu_v_aligned[i, 0]:.9f}",
                "imu_vy": f"{imu_v_aligned[i, 1]:.9f}",
                "gt_vx": f"{gt_v[i, 0]:.9f}",
                "gt_vy": f"{gt_v[i, 1]:.9f}",
                "imu_speed": f"{imu_speed[i]:.9f}",
                "gt_speed": f"{gt_speed[i]:.9f}",
                "imu_vel_heading_deg": f"{math.degrees(imu_vel_heading[i]):.6f}",
                "gt_motion_heading_deg": f"{math.degrees(gt_motion_heading[i]):.6f}",
                "vel_heading_error_deg": f"{math.degrees(heading_err[i]):.6f}",
                "imu_yaw_deg": f"{math.degrees(imu_yaw[i]):.6f}",
                "gt_pose_yaw_deg": f"{math.degrees(gt_yawm[i]):.6f}",
                "yaw_error_deg": f"{math.degrees(yaw_err[i]):.6f}",
                "ax_global": f"{imu_a_global_aligned[i, 0]:.9f}",
                "ay_global": f"{imu_a_global_aligned[i, 1]:.9f}",
                "az_global": f"{imu_a_global_aligned[i, 2]:.9f}",
                "gyro_x": f"{imu_g[i, 0]:.9f}",
                "gyro_y": f"{imu_g[i, 1]:.9f}",
                "gyro_z": f"{imu_g[i, 2]:.9f}",
                "accel_x": f"{imu_a_raw[i, 0]:.9f}",
                "accel_y": f"{imu_a_raw[i, 1]:.9f}",
                "accel_z": f"{imu_a_raw[i, 2]:.9f}",
                "backward_flag": int(backward_mask[i]),
            }
        )

    save_diagnostic_csv(output_dir / "backward_diagnostics.csv", rows)
    save_plots(
        output_dir,
        match_times - times[0],
        heading_err,
        yaw_err,
        imu_speed,
        gt_speed,
        (imu_p_aligned, gt_pm),
        backward_mask,
    )

    dt = np.diff(times)
    static_mask = times <= times[0] + args.static_duration
    accel_norm = np.linalg.norm(accels, axis=1)
    static_accel_norm = accel_norm[static_mask]
    static_gyro_norm = np.linalg.norm(gyros[static_mask], axis=1)
    dt_large = dt > max(0.05, 5.0 * np.median(dt)) if len(dt) else np.asarray([], dtype=bool)
    static_accel_norm_mean = float(np.mean(static_accel_norm)) if len(static_accel_norm) else float("nan")
    static_gyro_norm_mean = float(np.mean(static_gyro_norm)) if len(static_gyro_norm) else float("nan")
    max_abs_gyro = float(np.max(np.abs(gyros))) if len(gyros) else 0.0
    max_abs_time_diff_ms = float(np.max(np.abs(match_times - gt_match_times)) * 1000.0)

    data_flags = []
    if not (0.7 * args.gravity <= static_accel_norm_mean <= 1.3 * args.gravity):
        data_flags.append("static accel norm is far from gravity; check accel units or whether gravity was already removed")
    if static_gyro_norm_mean > 0.05:
        data_flags.append("static gyro norm is not close to zero; check static window, gyro units, or gyro bias")
    if np.count_nonzero(dt_large) > 0:
        data_flags.append("large IMU timestamp gaps found; integration can jump or freeze around those gaps")
    if max_abs_time_diff_ms > args.max_diff * 1000.0:
        data_flags.append("IMU/GT association time difference exceeds configured max-diff")
    if max_abs_gyro > 20.0:
        data_flags.append("very large gyro magnitude found; check whether gyro is deg/s instead of rad/s")

    lines = [
        "IMU backward diagnostics",
        f"bag: {Path(args.bag).resolve()}",
        f"imu_topic: {args.imu_topic}",
        f"odom_topic: {args.odom_topic}",
        f"imu_samples: {len(times)}",
        f"gt_samples: {len(gt_t)}",
        f"matched_samples: {len(imu_idx)}",
        f"gt_rate_only: {args.gt_rate_only}",
        f"duration_s: {times[-1] - times[0]:.6f}",
        f"align_mode: {args.align}",
        f"align_yaw_deg: {math.degrees(align_yaw):.6f}",
        "",
        "Raw timing checks",
        f"imu_dt: {percentile_text(dt, 's')}",
        f"large_dt_count: {int(np.count_nonzero(dt_large))}",
        f"nonpositive_dt_after_sort_count: {int(np.count_nonzero(dt <= 0.0))}",
        f"max_abs_imu_gt_time_diff_ms: {max_abs_time_diff_ms:.6f}",
        "",
        "Initial static-window checks",
        f"static_duration_s: {args.static_duration:.6f}",
        f"static_samples: {int(np.count_nonzero(static_mask))}",
        f"static_accel_norm: {percentile_text(static_accel_norm, 'm/s^2')}",
        f"static_gyro_norm: {percentile_text(static_gyro_norm, 'rad/s')}",
        f"gyro_bias: {gyro_bias[0]:.9f}, {gyro_bias[1]:.9f}, {gyro_bias[2]:.9f}",
        f"accel_bias: {accel_bias[0]:.9f}, {accel_bias[1]:.9f}, {accel_bias[2]:.9f}",
        "",
        "Whole-bag raw IMU checks",
        f"accel_norm: {percentile_text(accel_norm, 'm/s^2')}",
        f"gyro_abs_x: {percentile_text(np.abs(gyros[:, 0]), 'rad/s')}",
        f"gyro_abs_y: {percentile_text(np.abs(gyros[:, 1]), 'rad/s')}",
        f"gyro_abs_z: {percentile_text(np.abs(gyros[:, 2]), 'rad/s')}",
        "",
        "Data issue flags",
        *(data_flags if data_flags else ["none"]),
        "",
        "Matched trajectory checks",
        f"max_abs_velocity_heading_error_deg: {np.max(np.abs(np.degrees(heading_err))):.6f}",
        f"max_abs_yaw_error_deg: {np.max(np.abs(np.degrees(yaw_err))):.6f}",
        f"backward_threshold_deg: {args.backward_angle_deg:.6f}",
        f"backward_candidate_count: {int(np.count_nonzero(backward_mask))}",
    ]

    if first_segment is None:
        lines.extend(
            [
                "first_backward_segment: none",
                "initial_assessment: no sample met the configured backward threshold; lower --backward-angle-deg or speed thresholds if needed.",
            ]
        )
    else:
        s, e = first_segment
        local = slice(max(0, s - 20), min(len(match_times), e + 21))
        mean_yaw_err = np.mean(np.abs(np.degrees(yaw_err[local])))
        mean_vel_err = np.mean(np.abs(np.degrees(heading_err[local])))
        lines.extend(
            [
                f"first_backward_segment_indices: {s}..{e}",
                f"first_backward_segment_rel_time_s: {match_times[s] - times[0]:.6f}..{match_times[e] - times[0]:.6f}",
                f"first_backward_segment_duration_s: {match_times[e] - match_times[s]:.6f}",
                f"first_backward_imu_xy: {imu_p_aligned[s, 0]:.6f}, {imu_p_aligned[s, 1]:.6f}",
                f"first_backward_gt_xy: {gt_pm[s, 0]:.6f}, {gt_pm[s, 1]:.6f}",
                f"first_backward_imu_speed_mps: {imu_speed[s]:.6f}",
                f"first_backward_gt_speed_mps: {gt_speed[s]:.6f}",
                f"first_backward_vel_heading_error_deg: {math.degrees(heading_err[s]):.6f}",
                f"first_backward_yaw_error_deg: {math.degrees(yaw_err[s]):.6f}",
                f"near_segment_mean_abs_vel_heading_error_deg: {mean_vel_err:.6f}",
                f"near_segment_mean_abs_yaw_error_deg: {mean_yaw_err:.6f}",
            ]
        )
        if mean_yaw_err > 90.0:
            assessment = "initial_assessment: IMU yaw is also near-opposite to GT; suspect gyro sign/scale/bias, timestamp gaps, or frame convention."
        elif mean_vel_err > 120.0:
            assessment = "initial_assessment: velocity is opposite while yaw is not; suspect acceleration integration, gravity compensation, accel axis convention, or missing velocity constraints."
        else:
            assessment = "initial_assessment: backward flag is weak/local; inspect CSV around first_backward_segment."
        lines.append(assessment)

    lines.extend(
        [
            "",
            f"csv: {(output_dir / 'backward_diagnostics.csv').resolve()}",
            f"plot_heading_speed: {(output_dir / 'heading_speed_diagnostics.png').resolve()}",
            f"plot_xy: {(output_dir / 'xy_backward_candidates.png').resolve()}",
        ]
    )
    write_summary(output_dir / "summary.txt", lines)

    print("\n".join(lines))


if __name__ == "__main__":
    main()
