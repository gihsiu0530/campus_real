#!/usr/bin/env python3
"""Integrate a rosbag IMU topic without vision and optionally compare to odom."""

import argparse
import csv
import math
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import numpy as np
import rosbag


def skew(v):
    return np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])


def exp_so3(w):
    theta = np.linalg.norm(w)
    if theta < 1e-12:
        return np.eye(3) + skew(w)
    axis = w / theta
    k = skew(axis)
    return np.eye(3) + math.sin(theta) * k + (1.0 - math.cos(theta)) * (k @ k)


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


def rot_to_quat_wxyz(rot):
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
    q = np.array([qw, qx, qy, qz], dtype=float)
    return q / np.linalg.norm(q)


def rot_from_two_vectors(a, b):
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    a /= np.linalg.norm(a)
    b /= np.linalg.norm(b)
    v = np.cross(a, b)
    c = float(np.dot(a, b))
    if c > 1.0 - 1e-12:
        return np.eye(3)
    if c < -1.0 + 1e-12:
        axis = np.array([1.0, 0.0, 0.0])
        if abs(a[0]) > 0.9:
            axis = np.array([0.0, 1.0, 0.0])
        axis -= a * np.dot(a, axis)
        axis /= np.linalg.norm(axis)
        return exp_so3(math.pi * axis)
    k = skew(v)
    return np.eye(3) + k + k @ k * (1.0 / (1.0 + c))


def msg_time(msg, bag_time):
    stamp = getattr(getattr(msg, "header", None), "stamp", None)
    if stamp is not None and stamp.to_sec() > 0.0:
        return stamp.to_sec()
    return bag_time.to_sec()


def read_imu_bag(bag_path, topic):
    times = []
    gyros = []
    accels = []
    with rosbag.Bag(str(bag_path), "r") as bag:
        for _, msg, bag_time in bag.read_messages(topics=[topic]):
            times.append(msg_time(msg, bag_time))
            gyros.append([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
            accels.append([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
    if not times:
        raise RuntimeError(f"no IMU messages found for topic {topic} in {bag_path}")
    order = np.argsort(np.asarray(times))
    return np.asarray(times)[order], np.asarray(gyros, dtype=float)[order], np.asarray(accels, dtype=float)[order]


def pose_from_msg(msg):
    if hasattr(msg, "pose") and hasattr(msg.pose, "pose"):
        pose = msg.pose.pose
    elif hasattr(msg, "pose"):
        pose = msg.pose
    else:
        raise TypeError("GT topic must be nav_msgs/Odometry or geometry_msgs/PoseStamped")
    p = np.array([pose.position.x, pose.position.y, pose.position.z], dtype=float)
    q = np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w], dtype=float)
    return p, q


def yaw_from_rot(rot):
    return math.atan2(rot[1, 0], rot[0, 0])


def yaw_rot(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def read_zero_start_odom(bag_path, topic):
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
        raise RuntimeError(f"no GT messages found for topic {topic} in {bag_path}")
    order = np.argsort(np.asarray(times))
    times = np.asarray(times)[order]
    positions = np.asarray(positions, dtype=float)[order]
    quats = np.asarray(quats, dtype=float)[order]
    p0 = positions[0]
    r0_inv = yaw_rot(-yaw_from_rot(quat_xyzw_to_rot(quats[0])))
    return times, (r0_inv @ (positions - p0).T).T


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
        return np.eye(3), np.zeros(3)
    est_xy = est_p[:, :2] - est_p[0, :2]
    gt_xy = gt_p[:, :2] - gt_p[0, :2]
    cross = np.sum(est_xy[:, 0] * gt_xy[:, 1] - est_xy[:, 1] * gt_xy[:, 0])
    dot = np.sum(est_xy[:, 0] * gt_xy[:, 0] + est_xy[:, 1] * gt_xy[:, 1])
    yaw = math.atan2(cross, dot)
    rot = yaw_rot(yaw)
    return rot, gt_p[0] - rot @ est_p[0]


def integrate_imu(times, gyros, accels, static_duration, gravity, accel_bias_mode):
    t0 = times[0]
    static_mask = times <= t0 + static_duration
    if np.count_nonzero(static_mask) < 3:
        raise RuntimeError("not enough samples in static window; reduce --static-duration")

    gyro_bias = gyros[static_mask].mean(axis=0)
    accel_mean = accels[static_mask].mean(axis=0)
    accel_bias = np.zeros(3)
    if accel_bias_mode == "gravity_magnitude":
        accel_bias = accel_mean - gravity * accel_mean / np.linalg.norm(accel_mean)

    accel_init = accel_mean - accel_bias
    r_ito_g = rot_from_two_vectors(accel_init, np.array([0.0, 0.0, gravity]))
    gravity_vec = np.array([0.0, 0.0, gravity])

    positions = np.zeros((len(times), 3))
    velocities = np.zeros((len(times), 3))
    quats = np.zeros((len(times), 4))
    lin_accels = np.zeros((len(times), 3))
    quats[0] = rot_to_quat_wxyz(r_ito_g)

    for i in range(1, len(times)):
        dt = times[i] - times[i - 1]
        if dt <= 0.0 or dt > 1.0:
            positions[i] = positions[i - 1]
            velocities[i] = velocities[i - 1]
            quats[i] = quats[i - 1]
            continue
        omega = 0.5 * ((gyros[i - 1] - gyro_bias) + (gyros[i] - gyro_bias))
        r_ito_g = r_ito_g @ exp_so3(omega * dt)
        a_body = 0.5 * ((accels[i - 1] - accel_bias) + (accels[i] - accel_bias))
        a_global = r_ito_g @ a_body - gravity_vec
        velocities[i] = velocities[i - 1] + a_global * dt
        positions[i] = positions[i - 1] + velocities[i - 1] * dt + 0.5 * a_global * dt * dt
        lin_accels[i] = a_global
        quats[i] = rot_to_quat_wxyz(r_ito_g)

    return positions, velocities, quats, lin_accels, gyro_bias, accel_bias


def save_imu_csv(path, times, positions, velocities, quats, lin_accels):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "timestamp(ns)",
                "tx",
                "ty",
                "tz",
                "qw",
                "qx",
                "qy",
                "qz",
                "vx",
                "vy",
                "vz",
                "ax_global",
                "ay_global",
                "az_global",
            ]
        )
        for t, p, v, q, a in zip(times, positions, velocities, quats, lin_accels):
            writer.writerow(
                [
                    f"{t * 1e9:.0f}",
                    *[f"{x:.9f}" for x in p],
                    *[f"{x:.9f}" for x in q],
                    *[f"{x:.9f}" for x in v],
                    *[f"{x:.9f}" for x in a],
                ]
            )


def save_error_csv(path, times, imu_p, gt_p):
    diff = imu_p - gt_p
    xy_err = np.linalg.norm(diff[:, :2], axis=1)
    pos_err = np.linalg.norm(diff, axis=1)
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["timestamp(s)", "imu_x", "imu_y", "imu_z", "gt_x", "gt_y", "gt_z", "error_x_m", "error_y_m", "error_z_m", "xy_error_m", "position_error_m"])
        for t, pi, pg, d, exy, ep in zip(times, imu_p, gt_p, diff, xy_err, pos_err):
            writer.writerow([f"{t:.9f}", *[f"{x:.9f}" for x in pi], *[f"{x:.9f}" for x in pg], *[f"{x:.9f}" for x in d], f"{exy:.9f}", f"{ep:.9f}"])
    return xy_err, pos_err


def save_xy_plot(path, imu_p, gt_p=None):
    import matplotlib.pyplot as plt

    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(8, 8))
    if gt_p is not None:
        ax.plot(gt_p[:, 0], gt_p[:, 1], label="GT odom", linewidth=2.0)
    ax.plot(imu_p[:, 0], imu_p[:, 1], label="IMU dead reckoning", linewidth=2.0)
    ax.scatter(imu_p[0, 0], imu_p[0, 1], marker="x", s=70, label="IMU start")
    ax.scatter(imu_p[-1, 0], imu_p[-1, 1], marker="+", s=90, label="IMU end")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    # ax.set_title("Pure IMU Integration")
    ax.set_title("Camera-Integrated IMU")
    ax.axis("equal")
    ax.grid(True)
    ax.legend()
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def print_summary(name, values, unit):
    print(f"{name}: rmse={math.sqrt(np.mean(values ** 2)):.4f}{unit}, mean={np.mean(values):.4f}{unit}, median={np.median(values):.4f}{unit}, max={np.max(values):.4f}{unit}")


def main():
    parser = argparse.ArgumentParser(description="Pure IMU dead reckoning from a rosbag.")
    parser.add_argument("--bag", required=True, help="input rosbag path")
    parser.add_argument("--imu-topic", default="/external_imu/data", help="IMU topic")
    parser.add_argument("--odom-topic", default="", help="optional GT odom topic")
    parser.add_argument("--output", default="/home/cyc/open_vins/imu_dead_reckoning.csv", help="output IMU-only trajectory CSV")
    parser.add_argument("--error-csv", default="/home/cyc/open_vins/imu_odom_errors.csv", help="output error CSV when --odom-topic is set")
    parser.add_argument("--xy-plot", default="/home/cyc/open_vins/imu_gt_xy_overlay.png", help="output XY plot")
    parser.add_argument("--static-duration", type=float, default=3.0, help="initial stationary window for bias/gravity init [s]")
    parser.add_argument("--gravity", type=float, default=9.81, help="gravity magnitude [m/s^2]")
    parser.add_argument("--accel-bias-mode", choices=["none", "gravity_magnitude"], default="gravity_magnitude", help="initial accelerometer bias handling")
    parser.add_argument("--align", choices=["none", "posyaw"], default="posyaw", help="alignment to GT before error calculation")
    parser.add_argument("--max-diff", type=float, default=0.03, help="max timestamp difference for GT association [s]")
    args = parser.parse_args()

    times, gyros, accels = read_imu_bag(Path(args.bag), args.imu_topic)
    positions, velocities, quats, lin_accels, gyro_bias, accel_bias = integrate_imu(
        times, gyros, accels, args.static_duration, args.gravity, args.accel_bias_mode
    )
    save_imu_csv(args.output, times, positions, velocities, quats, lin_accels)

    gt_matched = None
    if args.odom_topic:
        gt_t, gt_p = read_zero_start_odom(Path(args.bag), args.odom_topic)
        imu_idx, gt_idx = associate_by_time(times, gt_t, args.max_diff)
        if len(imu_idx) < 3:
            raise RuntimeError(f"only {len(imu_idx)} matched poses; increase --max-diff or check timestamps")
        imu_p = positions[imu_idx]
        gt_matched = gt_p[gt_idx]
        rot, trans = align_positions(imu_p, gt_matched, args.align)
        imu_p_aligned = (rot @ imu_p.T).T + trans
        xy_err, pos_err = save_error_csv(args.error_csv, times[imu_idx], imu_p_aligned, gt_matched)
        save_xy_plot(args.xy_plot, imu_p_aligned, gt_matched)
        print(f"matched poses: {len(imu_idx)} / imu={len(times)}, gt={len(gt_t)}")
        print(f"alignment: {args.align}, max_diff={args.max_diff:.3f}s")
        print_summary("IMU-only XY position", xy_err, "m")
        print_summary("IMU-only 3D position", pos_err, "m")
        print(f"saved error CSV: {Path(args.error_csv).resolve()}")
    else:
        save_xy_plot(args.xy_plot, positions, gt_matched)

    print(f"IMU messages: {len(times)} from {args.imu_topic}")
    print(f"duration: {times[-1] - times[0]:.3f}s")
    print(f"gyro_bias: {gyro_bias[0]:.8f}, {gyro_bias[1]:.8f}, {gyro_bias[2]:.8f}")
    print(f"accel_bias: {accel_bias[0]:.8f}, {accel_bias[1]:.8f}, {accel_bias[2]:.8f}")
    print(f"final position: {positions[-1, 0]:.3f}, {positions[-1, 1]:.3f}, {positions[-1, 2]:.3f} m")
    print(f"saved IMU CSV:  {Path(args.output).resolve()}")
    print(f"saved XY plot:  {Path(args.xy_plot).resolve()}")


if __name__ == "__main__":
    main()
