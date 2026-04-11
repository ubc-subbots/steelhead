#!/usr/bin/env python3
"""Relay /steelhead/controls/ukf/odometry/filtered to /odom for the Simulink SMC controller."""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class OdomRelay(Node):
    def __init__(self):
        super().__init__('odom_relay')
        self.pub = self.create_publisher(Odometry, '/odom', 10)
        self.sub = self.create_subscription(
            Odometry,
            '/steelhead/controls/ukf/odometry/filtered',
            self.callback,
            10
        )

    def callback(self, msg):
        self.pub.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(OdomRelay())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
