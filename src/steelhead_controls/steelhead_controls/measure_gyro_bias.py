#!/usr/bin/env python3
"""Measure the D455 gyro bias for d455_imu_publisher's gyro_bias parameter.

The D455 has no magnetometer, so yaw is integrated open-loop from the gyro and
any bias on gyro z becomes yaw drift that nothing downstream can correct. This
averages a still sensor to estimate that bias.

Reads /camera/imu directly, so it measures the true raw bias no matter what
d455_imu_publisher's gyro_bias is currently set to. Only the camera needs to be
running:

    ros2 launch steelhead_controls imu_publisher_launch.py
    ros2 run steelhead_controls measure_gyro_bias.py --ros-args -p duration:=60.0

Hold the vehicle dead still for the whole run, and let the camera warm up ~10
minutes first: the bias shifts as the IMU heats, so a cold measurement will not
hold once it comes up to temperature.
"""

import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

from steelhead_controls.d455_imu_publisher import optical_to_body

# Above roughly this much sample-to-sample spread the vehicle was disturbed
# rather than sitting still; the BMI055's own noise is far below it
MOTION_STDDEV_LIMIT = 0.02  # rad/s


class GyroBiasMeasurer(Node):
    def __init__(self):
        super().__init__("measure_gyro_bias")

        self.declare_parameter("input_topic", "/camera/imu")
        self.declare_parameter("duration", 60.0)

        input_topic = self.get_parameter("input_topic").value
        self.duration = self.get_parameter("duration").value

        self.samples = []
        self.subscription_ = self.create_subscription(
            Imu, input_topic, self.imu_callback, qos_profile_sensor_data
        )

        self.get_logger().info(
            f"Averaging {input_topic} for {self.duration:.0f}s. Hold the vehicle still..."
        )

    def imu_callback(self, msg):
        self.samples.append(optical_to_body(msg.angular_velocity))


def mean_and_stddev(values):
    mean = sum(values) / len(values)
    variance = sum((v - mean) ** 2 for v in values) / len(values)
    return mean, math.sqrt(variance)


def main(args=None):
    rclpy.init(args=args)
    node = GyroBiasMeasurer()

    end = node.get_clock().now().nanoseconds + int(node.duration * 1e9)
    while rclpy.ok() and node.get_clock().now().nanoseconds < end:
        rclpy.spin_once(node, timeout_sec=0.1)

    samples = node.samples
    if len(samples) < 100:
        node.get_logger().error(
            f"Only {len(samples)} samples from "
            f"{node.get_parameter('input_topic').value}; is the camera running "
            f"with enable_gyro and unite_imu_method set?"
        )
        node.destroy_node()
        rclpy.shutdown()
        return 1

    axes = [mean_and_stddev([s[i] for s in samples]) for i in range(3)]
    bias = [m for m, _ in axes]

    print(f"\n{len(samples)} samples over {node.duration:.0f}s (body axes)\n")
    for name, (mean, stddev) in zip("xyz", axes):
        print(
            f"  gyro {name}: bias {mean:+.6f} rad/s ({math.degrees(mean):+.4f} deg/s)"
            f"   stddev {stddev:.6f} rad/s"
        )

    moved = any(stddev > MOTION_STDDEV_LIMIT for _, stddev in axes)
    if moved:
        print(
            "\nWARNING: sample spread is too high to be sensor noise alone. The "
            "vehicle was probably disturbed; this estimate is unreliable. Re-run "
            "with it held still."
        )

    drift = abs(math.degrees(bias[2])) * 120.0
    print(f"\nUncorrected, gyro z implies ~{drift:.1f} deg of yaw drift over 2 min.")
    print("Subtracting this bias leaves whatever the estimate itself is wrong by,")
    print("plus any drift as the IMU's temperature keeps changing.\n")
    print("Paste into imu_publisher_launch.py, d455_imu_publisher's parameters:\n")
    print(f"    'gyro_bias': [{bias[0]:.6f}, {bias[1]:.6f}, {bias[2]:.6f}],\n")

    node.destroy_node()
    rclpy.shutdown()
    return 1 if moved else 0


if __name__ == "__main__":
    sys.exit(main())
