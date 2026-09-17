#!/usr/bin/python3
"""
Relay OpenVINS /ov_msckf/poseimu (PoseWithCovarianceStamped) as nav_msgs/Odometry.

The FF planner and mpc_back_test both consume nav_msgs/Odometry. Publishing the
VIO pose in that type lets them switch from the LiDAR /odom to VIO by topic name
alone, so both stay in the same (VIO "global") frame.

The lever arm moves the reported point from the IMU to the vehicle reference
point, expressed in the IMU frame (x forward, y left, z up). The LiDAR that
defines /odom sits directly above the camera, so the default is zero.

Twist is left zero: the MPC takes speed from v_real and the planner reads pose only.
"""
import numpy as np
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_matrix


class VioPoseToOdom:
    def __init__(self):
        input_topic = rospy.get_param("~input_topic", "/ov_msckf/poseimu")
        output_topic = rospy.get_param("~output_topic", "/vio_odom")
        self.child_frame_id = rospy.get_param("~child_frame_id", "vio_base")
        self.lever_arm = np.array([
            float(rospy.get_param("~lever_arm_x", 0.0)),
            float(rospy.get_param("~lever_arm_y", 0.0)),
            float(rospy.get_param("~lever_arm_z", 0.0)),
        ])

        self.pub = rospy.Publisher(output_topic, Odometry, queue_size=10)
        self.sub = rospy.Subscriber(
            input_topic, PoseWithCovarianceStamped, self.cb_pose, queue_size=10
        )
        rospy.loginfo(
            f"[vio_pose_to_odom] {input_topic} -> {output_topic} "
            f"(child_frame_id={self.child_frame_id}, lever_arm={self.lever_arm.tolist()})"
        )

    def cb_pose(self, msg: PoseWithCovarianceStamped) -> None:
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation

        position = np.array([p.x, p.y, p.z])
        if np.any(self.lever_arm):
            rotation = quaternion_matrix([o.x, o.y, o.z, o.w])[:3, :3]
            position = position + rotation @ self.lever_arm

        odom = Odometry()
        odom.header = msg.header
        odom.child_frame_id = self.child_frame_id
        odom.pose.pose.position.x = float(position[0])
        odom.pose.pose.position.y = float(position[1])
        odom.pose.pose.position.z = float(position[2])
        odom.pose.pose.orientation = o
        odom.pose.covariance = msg.pose.covariance
        self.pub.publish(odom)


def main():
    rospy.init_node("vio_pose_to_odom")
    VioPoseToOdom()
    rospy.spin()


if __name__ == "__main__":
    main()
