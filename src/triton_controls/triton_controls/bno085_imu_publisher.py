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

        self.publisher_ = self.create_publisher(Imu, '/triton/drivers/imu/out', 10)
        # Timer: read and publish at ~100 Hz
        self.timer_ = self.create_timer(0.01, self.timer_callback)

    def timer_callback(self):
        try:
            # Attempt to read one line from serial
            line = self.ser.readline().decode('utf-8').strip()
            if not line:
                return  # no data

            tokens = line.split(',')
            # We expect 7 tokens: status,yaw,pitch,roll,ax,ay,az
            if len(tokens) < 7:
                # self.get_logger().warn(f'Not enough tokens in line: {line}')
                return

            try:
                # parse floats
                status = float(tokens[0])  # we might not use it, but let's parse
                if status != 3:
                    # self.get_logger().warn(f'Status is not 3: {status}. Skipping this message.')
                    return
                yaw_deg = float(tokens[1])
                pitch_deg = float(tokens[2])
                roll_deg = float(tokens[3])
                ax = float(tokens[4])
                ay = float(tokens[5])
                az = float(tokens[6])

                # Convert yaw/pitch/roll in degrees to quaternion (Z->Y->X)
                yaw_rad   = math.radians(yaw_deg)
                pitch_rad = math.radians(pitch_deg)
                roll_rad  = math.radians(roll_deg)

                cy = math.cos(yaw_rad * 0.5)
                sy = math.sin(yaw_rad * 0.5)
                cp = math.cos(pitch_rad * 0.5)
                sp = math.sin(pitch_rad * 0.5)
                cr = math.cos(roll_rad * 0.5)
                sr = math.sin(roll_rad * 0.5)

                qw = cr * cp * cy + sr * sp * sy
                qx = sr * cp * cy - cr * sp * sy
                qy = cr * sp * cy + sr * cp * sy
                qz = cr * cp * sy - sr * sp * cy

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

        except (serial.SerialException, UnicodeDecodeError) as e:
            self.get_logger().warn(f'Serial read error: {e}')
            return


def main(args=None):
    rclpy.init(args=args)
    node = Bno085ImuPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
