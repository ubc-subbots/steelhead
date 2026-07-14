#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


def optical_to_body(v):
    """Map a vector off the D455 optical axes onto REP-103 body axes.

    The camera reports on optical axes (x right, y down, z forward) however it
    is mounted; the body frame is x forward, y left, z up.
    """
    return (v.z, -v.x, -v.y)


class D455ImuPublisher(Node):
    """Re-express D455 IMU samples on the robot's body axes.

    The D455 has no fusion processor and no magnetometer, so these samples
    carry no orientation. imu_filter_madgwick fuses them downstream and is what
    actually publishes /steelhead/drivers/imu/out.

    Yaw is integrated open-loop from the gyro, so any bias on gyro z turns
    directly into yaw drift (roll and pitch stay pinned by gravity). Madgwick
    cannot estimate that bias without a heading reference, so it is subtracted
    here instead. Measure it with measure_gyro_bias.py.
    """

    def __init__(self):
        super().__init__("d455_imu_publisher")

        self.declare_parameter("input_topic", "/camera/imu")
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("gyro_bias", [0.0, 0.0, 0.0])

        input_topic = self.get_parameter("input_topic").value
        self.frame_id = self.get_parameter("frame_id").value

        self.gyro_bias = list(self.get_parameter("gyro_bias").value)
        if len(self.gyro_bias) != 3:
            raise ValueError(
                f"gyro_bias must be [x, y, z] in rad/s on body axes, got {self.gyro_bias}"
            )

        if any(self.gyro_bias):
            self.get_logger().info(
                f"Subtracting gyro bias (rad/s, body axes): {self.gyro_bias}"
            )
        else:
            self.get_logger().warn(
                "No gyro bias set; yaw will drift at the sensor's raw bias rate. "
                "Run measure_gyro_bias.py and set the gyro_bias parameter."
            )

        self.publisher_ = self.create_publisher(
            Imu, "/steelhead/drivers/imu/data_raw", 10
        )
        # The camera publishes IMU as best-effort sensor data; a reliable
        # subscription here would silently match nothing
        self.subscription_ = self.create_subscription(
            Imu, input_topic, self.imu_callback, qos_profile_sensor_data
        )

        self.get_logger().info(f"Remapping {input_topic} onto body axes...")

    def imu_callback(self, msg):
        out = Imu()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.frame_id

        ax, ay, az = optical_to_body(msg.linear_acceleration)
        out.linear_acceleration.x = ax
        out.linear_acceleration.y = ay
        out.linear_acceleration.z = az

        gx, gy, gz = optical_to_body(msg.angular_velocity)
        out.angular_velocity.x = gx - self.gyro_bias[0]
        out.angular_velocity.y = gy - self.gyro_bias[1]
        out.angular_velocity.z = gz - self.gyro_bias[2]

        # No orientation in this message; -1 is the sensor_msgs/Imu convention
        # for "this field is unset" and is what madgwick overwrites
        out.orientation_covariance[0] = -1.0

        self.publisher_.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = D455ImuPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
