#!/usr/bin/env python3
"""Extract an image topic from a rosbag and encode it to H.264 mp4 via ffmpeg.

Usage:
    python3 bag2video.py <bag>                       # auto-pick the image topic
    python3 bag2video.py <bag> -t /cam/image_raw     # pick topic explicitly
    python3 bag2video.py <bag> -o out.mp4 --fps 30 --crf 18
"""
import argparse
import os
import subprocess
import sys

import rosbag
from cv_bridge import CvBridge

IMAGE_TYPES = ("sensor_msgs/Image", "sensor_msgs/CompressedImage")


def pick_topic(bag, requested):
    topics = bag.get_type_and_topic_info().topics
    if requested:
        if requested not in topics:
            sys.exit("topic not in bag: %s" % requested)
        return requested
    candidates = [t for t, i in topics.items() if i.msg_type in IMAGE_TYPES]
    if not candidates:
        sys.exit("no image topic found in bag")
    if len(candidates) > 1:
        sys.exit("multiple image topics, pick one with -t:\n  " + "\n  ".join(candidates))
    return candidates[0]


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("bag")
    p.add_argument("-t", "--topic", help="image topic (auto-detected if omitted)")
    p.add_argument("-o", "--out", help="output mp4 (default: <bag>_<topic>.mp4)")
    p.add_argument("--fps", type=float, help="override fps (default: bag message rate)")
    p.add_argument("--crf", type=int, default=23, help="x264 quality, lower = better (default 23)")
    p.add_argument("--preset", default="medium", help="x264 preset (default medium)")
    args = p.parse_args()

    bag = rosbag.Bag(args.bag, "r")
    topic = pick_topic(bag, args.topic)
    info = bag.get_type_and_topic_info().topics[topic]
    count = info.message_count
    fps = args.fps or info.frequency or 30.0

    out = args.out
    if not out:
        stem = os.path.splitext(args.bag)[0]
        out = "%s_%s.mp4" % (stem, topic.strip("/").replace("/", "_"))

    print("topic : %s (%s)" % (topic, info.msg_type))
    print("frames: %d, fps: %.3f" % (count, fps))

    bridge = CvBridge()
    compressed = info.msg_type == "sensor_msgs/CompressedImage"
    proc = None
    written = 0

    for _, msg, _ in bag.read_messages(topics=[topic]):
        if compressed:
            frame = bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
        else:
            frame = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        if proc is None:
            h, w = frame.shape[:2]
            print("size  : %dx%d" % (w, h))
            print("out   : %s" % out)
            proc = subprocess.Popen(
                ["ffmpeg", "-y", "-loglevel", "error",
                 "-f", "rawvideo", "-pix_fmt", "bgr24",
                 "-s", "%dx%d" % (w, h), "-r", "%.6f" % fps,
                 "-i", "-",
                 "-c:v", "libx264", "-preset", args.preset, "-crf", str(args.crf),
                 "-pix_fmt", "yuv420p", out],
                stdin=subprocess.PIPE)
        proc.stdin.write(frame.tobytes())
        written += 1
        if written % 200 == 0:
            print("  %d / %d" % (written, count))
            sys.stdout.flush()

    bag.close()
    if proc is None:
        sys.exit("no messages on topic %s" % topic)
    proc.stdin.close()
    if proc.wait() != 0:
        sys.exit("ffmpeg failed")
    print("done: %d frames -> %s" % (written, out))


if __name__ == "__main__":
    main()
