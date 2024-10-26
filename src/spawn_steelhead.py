#!/usr/bin/env python3

import os

import rclpy
from rclpy.node import Node
from ros_gz_interfaces.srv import SpawnEntity

class SpawnSteelheadRobot(Node):
    def __init__(self):
        super().__init__('spawn_steelhead_robot')

        # Get paths
        pkg_steelhead = os.path.join('gazebo')
        urdf_file = os.path.join(pkg_steelhead, 'steelhead_urdf/urdf', 'steelhead_urdf.urdf')

        # Read URDF
        with open(urdf_file, 'r') as file:
            self.robot_description = file.read()

        # Create a client to call the spawn service
        self.cli = self.create_client(SpawnEntity, '/spawn_entity')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /spawn_entity service...')

        # Prepare the request
        self.req = SpawnEntity.Request()
        self.req.name = 'steelhead_robot'
        self.req.xml = self.robot_description
        self.req.robot_namespace = ''
        self.req.initial_pose.position.x = 0.0
        self.req.initial_pose.position.y = 0.0
        self.req.initial_pose.position.z = 0.0

        # Call the service
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)

        if self.future.result() is not None:
            self.get_logger().info('Successfully spawned steelhead_robot')
        else:
            self.get_logger().error('Failed to spawn steelhead_robot: %r' % self.future.exception())

        # Shutdown node after spawning
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    SpawnSteelheadRobot()

if __name__ == '__main__':
    main()
