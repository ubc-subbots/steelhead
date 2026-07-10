#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial
import math


class Bno085ImuPublisher(Node):
    def __init__(self):
        super().__init__('bno085_imu_publisher')

        port = '/dev/imu'
        baud = 115200 

        self.get_logger().info(f'Opening {port} at {baud} baud...')
        try:
            self.ser = serial.Serial(port, baud, timeout=0.01)
        except serial.serialutil.SerialException as e:
            self.get_logger().error(f'Failed to open port {port}: {e}')
            # Optionally rclpy.shutdown() or raise an exception
            raise

        self.publisher_ = self.create_publisher(Imu, '/steelhead/drivers/imu/out', 10)
        # Timer: read and publish at ~100 Hz
        self.timer_ = self.create_timer(0.01, self.timer_callback)

    def timer_callback(self):
        # Attempt to read one line from serial
        line = self.ser.readline().decode('utf-8').strip()
        if not line:
            return  # no data

        tokens = line.split(',')
        # We expect 8 tokens: status,qx,qy,qz,qw,ax,ay,az
        # The quaternion arrives in the BNO085's native i,j,k,real order,
        # matching bno085_serial_output_parser.ino
        if len(tokens) < 8:
            self.get_logger().warn(f'Not enough tokens in line: {line}')
            return

        try:
            # parse floats
            status = float(tokens[0])  # we might not use it, but let's parse
            qx = float(tokens[1])
            qy = float(tokens[2])
            qz = float(tokens[3])
            qw = float(tokens[4])
            ax = float(tokens[5])
            ay = float(tokens[6])
            az = float(tokens[7])

            # Renormalize: serial ascii truncates precision, and consumers
            # (tf2, robot_localization) expect a unit quaternion
            norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
            if norm < 1e-6:
                self.get_logger().warn(f'Zero-norm quaternion in line: {line}')
                return
            qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm

            # Construct and publish IMU message
            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'imu_link'

            msg.orientation.w = qw
            msg.orientation.x = qx
            msg.orientation.y = qy
            msg.orientation.z = qz

            msg.linear_acceleration.x = ax
            msg.linear_acceleration.y = ay
            msg.linear_acceleration.z = az

            # We are not using angular_velocity in this example

            self.publisher_.publish(msg)

        except ValueError:
            self.get_logger().warn(f'Failed to parse float from line: {line}')


def main(args=None):
    rclpy.init(args=args)
    node = Bno085ImuPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
