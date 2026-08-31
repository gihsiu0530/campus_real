#!/usr/bin/env python3
"""Convert sensor_msgs/Imu vectors from FRD to FLU.

FRD: x forward, y right, z down
FLU: x forward, y left,  z up

Only angular_velocity and linear_acceleration are used by OpenVINS. This node
converts those vectors and their covariances. Orientation is marked invalid by
default to avoid publishing a misleading attitude convention.
"""

import rospy
from sensor_msgs.msg import Imu


SIGNS = (1.0, -1.0, -1.0)


def convert_vector(vec):
    return vec.x, -vec.y, -vec.z


def convert_covariance(cov):
    if len(cov) != 9:
        return cov
    out = [0.0] * 9
    for r in range(3):
        for c in range(3):
            out[3 * r + c] = SIGNS[r] * SIGNS[c] * cov[3 * r + c]
    return out


class ImuFrdToFlu:
    def __init__(self):
        self.input_topic = rospy.get_param("~input", "/mavros/imu/data")
        self.output_topic = rospy.get_param("~output", "/mavros/imu/data_flu")
        self.output_frame = rospy.get_param("~frame_id", "base_link_flu")
        self.publish_orientation = rospy.get_param("~publish_orientation", False)

        self.pub = rospy.Publisher(self.output_topic, Imu, queue_size=200)
        self.sub = rospy.Subscriber(self.input_topic, Imu, self.callback, queue_size=200)
        rospy.loginfo("Converting IMU FRD -> FLU: %s -> %s", self.input_topic, self.output_topic)

    def callback(self, msg):
        out = Imu()
        out.header = msg.header
        out.header.frame_id = self.output_frame

        if self.publish_orientation:
            # If enabled, keep the orientation as-is. OpenVINS does not use it.
            # Leave this disabled unless the upstream attitude convention is known.
            out.orientation = msg.orientation
            out.orientation_covariance = convert_covariance(list(msg.orientation_covariance))
        else:
            out.orientation.w = 1.0
            out.orientation_covariance[0] = -1.0

        wx, wy, wz = convert_vector(msg.angular_velocity)
        out.angular_velocity.x = wx
        out.angular_velocity.y = wy
        out.angular_velocity.z = wz
        out.angular_velocity_covariance = convert_covariance(list(msg.angular_velocity_covariance))

        ax, ay, az = convert_vector(msg.linear_acceleration)
        out.linear_acceleration.x = ax
        out.linear_acceleration.y = ay
        out.linear_acceleration.z = az
        out.linear_acceleration_covariance = convert_covariance(list(msg.linear_acceleration_covariance))

        self.pub.publish(out)


def main():
    rospy.init_node("imu_frd_to_flu")
    ImuFrdToFlu()
    rospy.spin()


if __name__ == "__main__":
    main()
