#!/usr/bin/env python3
"""Convert a NavSatFix topic in a ROS1 bag to OpenVINS/EuRoC-style groundtruth.

The output is a 17-column CSV:
timestamp(ns), p_RS_R_x, p_RS_R_y, p_RS_R_z, q_RS_w, q_RS_x, q_RS_y, q_RS_z,
v_RS_R_x, v_RS_R_y, v_RS_R_z, b_w_RS_S_x, b_w_RS_S_y, b_w_RS_S_z,
b_a_RS_S_x, b_a_RS_S_y, b_a_RS_S_z

Only position comes from NavSatFix. Orientation is identity, biases are zero,
and velocity is estimated by finite differences in local ENU coordinates.
This is useful for plotting or rough trajectory comparison, not for calibrated
attitude groundtruth.
"""


# source devel/setup.bash
# python3 src/open_vins/ov_eval/python/navsatfix_to_openvins_gt.py \
#   /home/cyc/open_vins/data/漢翔/2026_03_25-15_34_03_0.bag \
#   --output /home/cyc/open_vins/gt_hanxiang_drone.csv
  
  
import argparse
import csv
import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

import rosbag


WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)


@dataclass
class Fix:
    stamp: float
    lat: float
    lon: float
    alt: float


def geodetic_to_ecef(lat_deg: float, lon_deg: float, alt_m: float) -> Tuple[float, float, float]:
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    sin_lon = math.sin(lon)
    cos_lon = math.cos(lon)
    n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    x = (n + alt_m) * cos_lat * cos_lon
    y = (n + alt_m) * cos_lat * sin_lon
    z = (n * (1.0 - WGS84_E2) + alt_m) * sin_lat
    return x, y, z


def ecef_to_enu(
    x: float,
    y: float,
    z: float,
    ref_lat_deg: float,
    ref_lon_deg: float,
    ref_ecef: Tuple[float, float, float],
) -> Tuple[float, float, float]:
    lat = math.radians(ref_lat_deg)
    lon = math.radians(ref_lon_deg)
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    sin_lon = math.sin(lon)
    cos_lon = math.cos(lon)

    dx = x - ref_ecef[0]
    dy = y - ref_ecef[1]
    dz = z - ref_ecef[2]

    east = -sin_lon * dx + cos_lon * dy
    north = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz
    up = cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz
    return east, north, up


def read_fixes(bag_path: str, topic: str) -> List[Fix]:
    fixes: List[Fix] = []
    with rosbag.Bag(bag_path, "r") as bag:
        for _, msg, _ in bag.read_messages(topics=[topic]):
            if msg.status.status < 0:
                continue
            if not all(math.isfinite(v) for v in (msg.latitude, msg.longitude, msg.altitude)):
                continue
            fixes.append(Fix(msg.header.stamp.to_sec(), msg.latitude, msg.longitude, msg.altitude))
    if not fixes:
        raise RuntimeError(f"No valid NavSatFix messages found on {topic}")
    return fixes


def finite_difference(values: List[Tuple[float, float, float, float]]) -> List[Tuple[float, float, float]]:
    velocities: List[Tuple[float, float, float]] = []
    for i, item in enumerate(values):
        if len(values) == 1:
            velocities.append((0.0, 0.0, 0.0))
            continue
        if i == 0:
            prev_item = values[i]
            next_item = values[i + 1]
        elif i == len(values) - 1:
            prev_item = values[i - 1]
            next_item = values[i]
        else:
            prev_item = values[i - 1]
            next_item = values[i + 1]
        dt = next_item[0] - prev_item[0]
        if dt <= 0.0:
            velocities.append((0.0, 0.0, 0.0))
            continue
        velocities.append(
            (
                (next_item[1] - prev_item[1]) / dt,
                (next_item[2] - prev_item[2]) / dt,
                (next_item[3] - prev_item[3]) / dt,
            )
        )
    return velocities


def convert(
    bag_path: str,
    output_path: str,
    topic: str,
    ref_lat: Optional[float],
    ref_lon: Optional[float],
    ref_alt: Optional[float],
) -> None:
    fixes = read_fixes(bag_path, topic)
    first = fixes[0]
    origin_lat = first.lat if ref_lat is None else ref_lat
    origin_lon = first.lon if ref_lon is None else ref_lon
    origin_alt = first.alt if ref_alt is None else ref_alt
    ref_ecef = geodetic_to_ecef(origin_lat, origin_lon, origin_alt)

    positions: List[Tuple[float, float, float, float]] = []
    for fix in fixes:
        ecef = geodetic_to_ecef(fix.lat, fix.lon, fix.alt)
        east, north, up = ecef_to_enu(ecef[0], ecef[1], ecef[2], origin_lat, origin_lon, ref_ecef)
        positions.append((fix.stamp, east, north, up))

    velocities = finite_difference(positions)

    with open(output_path, "w", newline="") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(
            [
                "#timestamp",
                "p_RS_R_x [m]",
                "p_RS_R_y [m]",
                "p_RS_R_z [m]",
                "q_RS_w []",
                "q_RS_x []",
                "q_RS_y []",
                "q_RS_z []",
                "v_RS_R_x [m s^-1]",
                "v_RS_R_y [m s^-1]",
                "v_RS_R_z [m s^-1]",
                "b_w_RS_S_x [rad s^-1]",
                "b_w_RS_S_y [rad s^-1]",
                "b_w_RS_S_z [rad s^-1]",
                "b_a_RS_S_x [m s^-2]",
                "b_a_RS_S_y [m s^-2]",
                "b_a_RS_S_z [m s^-2]",
            ]
        )
        for (stamp, east, north, up), (vx, vy, vz) in zip(positions, velocities):
            writer.writerow(
                [
                    f"{stamp * 1e9:.0f}",
                    f"{east:.9f}",
                    f"{north:.9f}",
                    f"{up:.9f}",
                    "1.0",
                    "0.0",
                    "0.0",
                    "0.0",
                    f"{vx:.9f}",
                    f"{vy:.9f}",
                    f"{vz:.9f}",
                    "0.0",
                    "0.0",
                    "0.0",
                    "0.0",
                    "0.0",
                    "0.0",
                ]
            )

    print(f"Wrote {len(positions)} poses to {output_path}")
    print(f"ENU origin: lat={origin_lat:.10f}, lon={origin_lon:.10f}, alt={origin_alt:.3f}")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", help="Input ROS1 bag")
    parser.add_argument("--topic", default="/mavros/global_position/global", help="NavSatFix topic")
    parser.add_argument("--output", required=True, help="Output CSV path")
    parser.add_argument("--ref-lat", type=float, default=None, help="Reference latitude. Defaults to first fix.")
    parser.add_argument("--ref-lon", type=float, default=None, help="Reference longitude. Defaults to first fix.")
    parser.add_argument("--ref-alt", type=float, default=None, help="Reference altitude. Defaults to first fix.")
    args = parser.parse_args()

    convert(args.bag, args.output, args.topic, args.ref_lat, args.ref_lon, args.ref_alt)


if __name__ == "__main__":
    main()
