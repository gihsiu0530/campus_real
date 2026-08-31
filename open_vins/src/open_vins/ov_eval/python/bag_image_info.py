#!/usr/bin/env python3
"""Print basic sensor_msgs/Image information from a ROS1 bag."""

import argparse
from collections import Counter

import rosbag


def inspect_images(bag_path, topic, max_messages):
    widths = Counter()
    heights = Counter()
    encodings = Counter()
    steps = Counter()
    stamps = []
    count = 0

    with rosbag.Bag(bag_path, "r") as bag:
        for _, msg, _ in bag.read_messages(topics=[topic]):
            widths[msg.width] += 1
            heights[msg.height] += 1
            encodings[msg.encoding] += 1
            steps[msg.step] += 1
            stamps.append(msg.header.stamp.to_sec())
            count += 1
            if max_messages and count >= max_messages:
                break

    if count == 0:
        raise RuntimeError(f"No Image messages found on {topic}")

    duration = stamps[-1] - stamps[0] if len(stamps) > 1 else 0.0
    hz = (len(stamps) - 1) / duration if duration > 0.0 else 0.0

    print(f"bag: {bag_path}")
    print(f"topic: {topic}")
    print(f"messages inspected: {count}")
    print(f"first stamp: {stamps[0]:.9f}")
    print(f"last stamp: {stamps[-1]:.9f}")
    print(f"duration: {duration:.3f} sec")
    print(f"estimated hz: {hz:.3f}")
    print(f"widths: {dict(widths)}")
    print(f"heights: {dict(heights)}")
    print(f"encodings: {dict(encodings)}")
    print(f"steps: {dict(steps)}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", help="Input ROS1 bag")
    parser.add_argument("--topic", default="/image_raw", help="Image topic")
    parser.add_argument("--max-messages", type=int, default=0, help="0 means inspect all messages")
    args = parser.parse_args()
    inspect_images(args.bag, args.topic, args.max_messages)


if __name__ == "__main__":
    main()
