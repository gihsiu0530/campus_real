#!/usr/bin/env python3
"""
Simulation stand-in for realtime_planner_node_ff_VIO.py.

The real planner publishes, every 0.5 s, a short path in front of the vehicle
on /senpai/array_topic as Float64MultiArray [x0, y0, x1, y1, ...]:
  - point 0: one collinear point behind the start (path_back_extension_m)
  - point 1: the current vehicle pose
  - then points every path_point_spacing_m (0.7 m) along the prediction,
    3 s long, extrapolated to at least path_min_points (7) points.
With ~ego_input_mode fixed_speed the prediction length is fixed_speed * 3 s;
with real_odom it is the measured speed * 3 s.

This node builds the same kind of path from the CSV route (global_path on
array_topic) instead of a learned model, so the MPC's planner mode can be
tested in mpc_simulate. The path starts at the vehicle pose and its lateral
offset from the route decays linearly to zero at the far end, roughly like a
prediction that heads back to the road centre.
"""
import math

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Float64MultiArray


def savitzky_golay2(v, m):
    """Quadratic Savitzky-Golay smoothing, same as mpc.cpp savitzkyGolay2."""
    n = len(v)
    out = v.copy()
    for i in range(n):
        r = min(i, n - 1 - i, m)
        if r < 2:
            continue
        j = np.arange(-r, r + 1)
        c = (3 * (3 * r * r + 3 * r - 1) - 15 * j * j) / (
            (2 * r + 1) * (4 * r * r + 4 * r - 3))
        out[i] = float(np.dot(c, v[i - r:i + r + 1]))
    return out


class FakePlanner:
    def __init__(self):
        self.mode = rospy.get_param("~ego_input_mode", "fixed_speed")
        if self.mode not in ("fixed_speed", "real_odom"):
            raise ValueError("~ego_input_mode must be fixed_speed or real_odom")
        self.fixed_speed = float(rospy.get_param("~fixed_speed_mps", 1.0))
        self.horizon_s = float(rospy.get_param("~horizon_s", 3.0))
        self.spacing = float(rospy.get_param("~path_point_spacing_m", 0.7))
        self.min_points = int(rospy.get_param("~path_min_points", 7))
        self.back_ext = float(rospy.get_param("~path_back_extension_m", self.spacing))
        self.smooth_window = int(rospy.get_param("~route_smooth_window", 5))
        period = float(rospy.get_param("~period_s", 0.5))

        self.route = None  # (N, 2) smoothed route
        self.route_s = None
        self.last_idx = None
        self.pose = None  # (x, y, speed)

        self.pub = rospy.Publisher(rospy.get_param("~out_topic", "/senpai/array_topic"),
                                   Float64MultiArray, queue_size=1)
        # Same path as nav_msgs/Path for RViz (the real planner publishes it on
        # /senpai/path_global too).
        self.path_pub = rospy.Publisher(
            rospy.get_param("~path_global_topic", "/senpai/path_global"),
            Path, queue_size=1)
        self.frame_id = rospy.get_param("~frame_id", "map")
        rospy.Subscriber(rospy.get_param("~route_topic", "array_topic"),
                         Float64MultiArray, self.route_cb, queue_size=1)
        rospy.Subscriber(rospy.get_param("~pose_topic", "/mpc_new_pose"),
                         Odometry, self.pose_cb, queue_size=1)
        rospy.Timer(rospy.Duration(period), self.tick)
        rospy.loginfo("[fake planner] mode %s, horizon %.1f s, spacing %.2f m, "
                      "min %d points", self.mode, self.horizon_s, self.spacing,
                      self.min_points)

    def route_cb(self, msg):
        if self.route is not None or len(msg.data) < 8:
            return
        pts = np.asarray(msg.data, dtype=float).reshape(-1, 2)
        if self.smooth_window > 0:
            pts = np.c_[savitzky_golay2(pts[:, 0], self.smooth_window),
                        savitzky_golay2(pts[:, 1], self.smooth_window)]
        self.route = pts
        self.route_s = np.r_[0.0, np.cumsum(np.hypot(*np.diff(pts, axis=0).T))]
        rospy.loginfo("[fake planner] route: %d points, %.1f m", len(pts), self.route_s[-1])

    def pose_cb(self, msg):
        p = msg.pose.pose.position
        v = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        self.pose = (p.x, p.y, v)

    def point_at(self, s):
        s = min(max(s, 0.0), self.route_s[-1])
        i = int(np.clip(np.searchsorted(self.route_s, s) - 1, 0, len(self.route) - 2))
        seg = self.route_s[i + 1] - self.route_s[i]
        f = (s - self.route_s[i]) / seg if seg > 1e-9 else 0.0
        return self.route[i] + f * (self.route[i + 1] - self.route[i])

    def project(self, x, y):
        """Arc length of the vehicle's projection on the route. The route is a
        loop (start ~ end), so search only near the last match."""
        n = len(self.route)
        if self.last_idx is None:
            lo, hi = 0, max(2, n // 5)
        else:
            lo, hi = max(0, self.last_idx - 5), min(n - 1, self.last_idx + 40)
        a = self.route[lo:hi]
        d = self.route[lo + 1:hi + 1] - a
        dd = np.maximum((d ** 2).sum(1), 1e-12)
        t = np.clip(((x - a[:, 0]) * d[:, 0] + (y - a[:, 1]) * d[:, 1]) / dd, 0.0, 1.0)
        proj = a + t[:, None] * d
        k = int(np.argmin(np.hypot(proj[:, 0] - x, proj[:, 1] - y)))
        self.last_idx = lo + k
        return self.route_s[lo + k] + t[k] * math.sqrt(dd[k])

    def tick(self, _event):
        if self.route is None or self.pose is None:
            return
        x, y, v = self.pose
        s0 = self.project(x, y)

        speed = self.fixed_speed if self.mode == "fixed_speed" else v
        length = max(speed * self.horizon_s, (self.min_points - 1) * self.spacing)
        length = min(length, self.route_s[-1] - s0)  # the route ends
        n_seg = int(length / self.spacing + 1e-6)  # 4.2 / 0.7 is 5.999...
        if n_seg < 2:
            # Near the end of the route: stop publishing, the MPC's PLAN_TIMEOUT
            # then commands a stop (like the real planner going silent).
            rospy.loginfo_throttle(5.0, "[fake planner] end of route, not publishing")
            return

        offset = np.array([x, y]) - self.point_at(s0)
        pts = [np.array([x, y])]
        for k in range(1, n_seg + 1):
            w = 1.0 - k / float(n_seg)  # lateral offset decays to 0 at the end
            pts.append(self.point_at(s0 + k * self.spacing) + w * offset)
        pts = np.asarray(pts)
        if self.back_ext > 0.0:
            u = pts[1] - pts[0]
            u = u / max(np.linalg.norm(u), 1e-9)
            pts = np.vstack([pts[0] - self.back_ext * u, pts])

        self.pub.publish(Float64MultiArray(data=pts.reshape(-1).tolist()))

        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = self.frame_id
        for i, (px, py) in enumerate(pts):
            nxt = pts[min(i + 1, len(pts) - 1)] - pts[max(i - 1, 0)]
            yaw = math.atan2(nxt[1], nxt[0])
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = float(px)
            ps.pose.position.y = float(py)
            ps.pose.orientation.z = math.sin(yaw / 2.0)
            ps.pose.orientation.w = math.cos(yaw / 2.0)
            path.poses.append(ps)
        self.path_pub.publish(path)


if __name__ == "__main__":
    rospy.init_node("fake_planner")
    FakePlanner()
    rospy.spin()
