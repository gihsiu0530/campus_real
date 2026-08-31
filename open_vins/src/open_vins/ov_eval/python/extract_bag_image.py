#!/usr/bin/env python3
"""Extract one sensor_msgs/Image from a ROS1 bag as a PNG."""

import argparse

import cv2
import rosbag
from cv_bridge import CvBridge


def extract_image(bag_path, topic, output_path, index):
    bridge = CvBridge()
    count = 0
    with rosbag.Bag(bag_path, "r") as bag:
        for _, msg, _ in bag.read_messages(topics=[topic]):
            if count == index:
                if msg.encoding in ("rgb8", "rgba8"):
                    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                else:
                    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                if not cv2.imwrite(output_path, cv_image):
                    raise RuntimeError(f"Failed to write {output_path}")
                print(f"Wrote {output_path}")
                print(f"stamp: {msg.header.stamp.to_sec():.9f}")
                print(f"width: {msg.width}")
                print(f"height: {msg.height}")
                print(f"encoding: {msg.encoding}")
                return
            count += 1
    raise RuntimeError(f"Topic {topic} has fewer than {index + 1} messages")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", help="Input ROS1 bag")
    parser.add_argument("--topic", default="/image_raw", help="Image topic")
    parser.add_argument("--output", required=True, help="Output PNG path")
    parser.add_argument("--index", type=int, default=0, help="Zero-based image index to extract")
    args = parser.parse_args()
    extract_image(args.bag, args.topic, args.output, args.index)


if __name__ == "__main__":
    main()
