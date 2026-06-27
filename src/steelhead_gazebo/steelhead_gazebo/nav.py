#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench


class NavNode(Node):
    def __init__(self):
        super().__init__('nav')

        self.declare_parameter('topic', '/steelhead/controls/hover_adjust')
        self.declare_parameter('rate', 10.0)
        self.declare_parameter('force_x', 15.0)
        self.declare_parameter('force_y', 0.0)
        self.declare_parameter('force_z', 0.0)
        self.declare_parameter('torque_x', 0.0)
        self.declare_parameter('torque_y', 0.0)
        self.declare_parameter('torque_z', 0.0)

        topic = self.get_parameter('topic').value
        self.publisher_ = self.create_publisher(Wrench, topic, 10)

        rate = self.get_parameter('rate').value
        self.timer = self.create_timer(1.0 / rate, self.publish_wrench)

        self.get_logger().info(f"Publishing constant wrench to '{topic}' at {rate} Hz")

    def publish_wrench(self):
        msg = Wrench()
        msg.force.x = self.get_parameter('force_x').value
        msg.force.y = self.get_parameter('force_y').value
        msg.force.z = self.get_parameter('force_z').value
        msg.torque.x = self.get_parameter('torque_x').value
        msg.torque.y = self.get_parameter('torque_y').value
        msg.torque.z = self.get_parameter('torque_z').value
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = NavNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
