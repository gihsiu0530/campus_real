#!/usr/bin/env python3
"""Run OpenVINS against generated Hanxiang candidate configs and score logs.

Run this from the workspace root after sourcing devel/setup.bash, or pass
--setup to source the workspace inside each command.
"""

import argparse
import os
import re
import signal
import subprocess
import time
from pathlib import Path


DIST_RE = re.compile(r"dist = ([0-9.+\-eE]+) \(meters\)")
STATE_RE = re.compile(r"p_IinG = ([^|]+) \| dist = ([0-9.+\-eE]+) \(meters\)")


def terminate_process(proc, timeout=8.0):
    if proc.poll() is not None:
        return
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
    except ProcessLookupError:
        return
    deadline = time.time() + timeout
    while time.time() < deadline:
        if proc.poll() is not None:
            return
        time.sleep(0.2)
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
    except ProcessLookupError:
        pass


def command_with_setup(command, setup):
    if setup:
        return ["bash", "-lc", f"source {setup} && {command}"]
    return ["bash", "-lc", command]


def score_log(log_path):
    text = Path(log_path).read_text(errors="replace")
    distances = [float(match.group(1)) for match in DIST_RE.finditer(text)]
    initialized = bool(distances)
    max_dist = max(distances) if distances else float("inf")
    final_dist = distances[-1] if distances else float("inf")
    huge_count = sum(1 for value in distances if value > 1000.0)
    return initialized, final_dist, max_dist, huge_count, len(distances)


def run_candidate(config, bag, args, log_dir):
    log_path = log_dir / f"{config}.log"
    estimate_path = log_dir / f"{config}_estimate.csv"

    launch_cmd = (
        "roslaunch ov_msckf subscribe.launch "
        f"config:={config} max_cameras:=1 use_stereo:=false dolivetraj:=false "
        f"dosave:=true path_est:={estimate_path}"
    )
    bag_cmd = f"rosbag play {bag} --clock"

    with open(log_path, "w") as log_file:
        launch_proc = subprocess.Popen(
            command_with_setup(launch_cmd, args.setup),
            stdout=log_file,
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,
        )
        time.sleep(args.launch_wait)
        bag_proc = subprocess.Popen(
            command_with_setup(bag_cmd, args.setup),
            stdout=log_file,
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,
        )
        deadline = time.time() + args.timeout
        while time.time() < deadline:
            if launch_proc.poll() is not None:
                break
            if bag_proc.poll() is not None:
                break
            time.sleep(0.5)
        terminate_process(bag_proc)
        terminate_process(launch_proc)

    initialized, final_dist, max_dist, huge_count, count = score_log(log_path)
    return {
        "config": config,
        "initialized": initialized,
        "final_dist": final_dist,
        "max_dist": max_dist,
        "huge_count": huge_count,
        "updates": count,
        "log": str(log_path),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True, help="Short bag to play")
    parser.add_argument("--config-prefix", default="hanxiang_drone_cand_", help="Candidate config folder prefix")
    parser.add_argument("--config-dir", default="src/open_vins/config")
    parser.add_argument("--log-dir", default="/tmp/hanxiang_candidate_logs")
    parser.add_argument("--timeout", type=float, default=70.0, help="Seconds to allow each run")
    parser.add_argument("--launch-wait", type=float, default=3.0)
    parser.add_argument("--setup", default="", help="Optional setup.bash path, e.g. /home/cyc/open_vins/devel/setup.bash")
    args = parser.parse_args()

    config_dir = Path(args.config_dir)
    configs = sorted(path.name for path in config_dir.iterdir() if path.is_dir() and path.name.startswith(args.config_prefix))
    if not configs:
        raise RuntimeError(f"No candidate configs found with prefix {args.config_prefix} in {config_dir}")

    log_dir = Path(args.log_dir)
    log_dir.mkdir(parents=True, exist_ok=True)

    results = []
    for index, config in enumerate(configs, 1):
        print(f"[{index}/{len(configs)}] testing {config}")
        result = run_candidate(config, args.bag, args, log_dir)
        results.append(result)
        print(
            "  initialized={initialized} updates={updates} final_dist={final_dist:.3f} "
            "max_dist={max_dist:.3f} huge_count={huge_count} log={log}".format(**result)
        )

    results.sort(key=lambda item: (not item["initialized"], item["huge_count"], item["max_dist"], item["final_dist"]))
    summary_path = log_dir / "summary.csv"
    with open(summary_path, "w") as f:
        f.write("config,initialized,updates,final_dist,max_dist,huge_count,log\n")
        for item in results:
            f.write(
                f"{item['config']},{int(item['initialized'])},{item['updates']},"
                f"{item['final_dist']},{item['max_dist']},{item['huge_count']},{item['log']}\n"
            )
    print(f"\nSummary: {summary_path}")
    print("Best candidates:")
    for item in results[:5]:
        print(
            "  {config}: initialized={initialized} updates={updates} final_dist={final_dist:.3f} "
            "max_dist={max_dist:.3f} huge_count={huge_count}".format(**item)
        )


if __name__ == "__main__":
    main()
