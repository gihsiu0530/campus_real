#!/usr/bin/env python3
"""Copy a time window from a ROS1 bag into a smaller bag."""

import argparse

import rosbag


def trim_bag(input_bag, output_bag, start_offset, duration):
    with rosbag.Bag(input_bag, "r") as src:
        start_time = src.get_start_time() + start_offset
        end_time = start_time + duration
        copied = 0
        with rosbag.Bag(output_bag, "w") as dst:
            for topic, msg, t in src.read_messages():
                stamp = t.to_sec()
                if stamp < start_time:
                    continue
                if stamp > end_time:
                    break
                dst.write(topic, msg, t)
                copied += 1
    print(f"Wrote {copied} messages to {output_bag}")
    print(f"window: {start_time:.9f} to {end_time:.9f} ({duration:.3f} sec)")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input_bag")
    parser.add_argument("--output", required=True)
    parser.add_argument("--start-offset", type=float, default=0.0, help="Seconds after bag start")
    parser.add_argument("--duration", type=float, default=50.0, help="Seconds to copy")
    args = parser.parse_args()
    trim_bag(args.input_bag, args.output, args.start_offset, args.duration)


if __name__ == "__main__":
    main()
