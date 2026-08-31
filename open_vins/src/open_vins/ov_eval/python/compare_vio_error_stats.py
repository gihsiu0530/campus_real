#!/usr/bin/env python3
"""Compare VIO error CSV statistics for x, y, and XY-plane errors."""

import argparse
import csv
import math
from pathlib import Path


DEFAULT_FILES = [
    (
        "builtin_imu",
        "/home/cyc/open_vins/內建imu成功/vio_odom_errors.csv",
    ),
    (
        "external_imu_yaw14p3_config",
        "/home/cyc/open_vins/vio_odom_errors_imu_ms_mono_yaw15p6.csv",
    ),
]


def finite_float(text):
    try:
        value = float(text)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(value):
        return None
    return value


def read_errors(path):
    path = Path(path)
    if not path.exists():
        raise FileNotFoundError(f"file does not exist: {path}")

    error_x = []
    error_y = []
    xy_error = []

    with path.open(newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            raise RuntimeError(f"empty CSV: {path}")
        missing = [name for name in ("error_x_m", "error_y_m") if name not in reader.fieldnames]
        if missing:
            raise RuntimeError(f"{path} missing required columns: {', '.join(missing)}")

        for row in reader:
            ex = finite_float(row.get("error_x_m"))
            ey = finite_float(row.get("error_y_m"))
            if ex is None or ey is None:
                continue
            error_x.append(abs(ex))
            error_y.append(abs(ey))

            exy = finite_float(row.get("xy_error_m"))
            if exy is None:
                exy = math.hypot(ex, ey)
            xy_error.append(exy)

    if not error_x:
        raise RuntimeError(f"no valid error rows found in {path}")
    return error_x, error_y, xy_error


def stats(values):
    n = len(values)
    avg = sum(values) / n
    rmse = math.sqrt(sum(v * v for v in values) / n)
    return {
        "n": n,
        "min": min(values),
        "max": max(values),
        "avg": avg,
        "rmse": rmse,
    }


def print_stats(label, path, error_x, error_y, xy_error):
    all_stats = {
        "abs_x_error_m": stats(error_x),
        "abs_y_error_m": stats(error_y),
        "xy_error_m": stats(xy_error),
    }
    print(f"\n[{label}]")
    print(f"file: {Path(path).resolve()}")
    print(f"rows: {len(error_x)}")
    print(f"{'metric':<12} {'min(m)':>12} {'max(m)':>12} {'avg(m)':>12} {'rmse(m)':>12}")
    for metric, s in all_stats.items():
        print(
            f"{metric:<12} "
            f"{s['min']:12.6f} "
            f"{s['max']:12.6f} "
            f"{s['avg']:12.6f} "
            f"{s['rmse']:12.6f}"
        )


def parse_input(items):
    if not items:
        return DEFAULT_FILES

    parsed = []
    for idx, item in enumerate(items, start=1):
        if "=" in item:
            label, path = item.split("=", 1)
            label = label.strip() or f"file{idx}"
            path = path.strip()
        else:
            label = f"file{idx}"
            path = item
        parsed.append((label, path))
    return parsed


def main():
    parser = argparse.ArgumentParser(description="Compute x/y/XY error min, max, average, and RMSE from VIO error CSVs.")
    parser.add_argument(
        "csv",
        nargs="*",
        help="optional CSVs as path or label=path; defaults compare builtin IMU and external IMU yaw config",
    )
    args = parser.parse_args()

    for label, path in parse_input(args.csv):
        error_x, error_y, xy_error = read_errors(path)
        print_stats(label, path, error_x, error_y, xy_error)


if __name__ == "__main__":
    main()
