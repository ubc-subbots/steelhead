#!/usr/bin/env python3

# Host-side parser for the MS5837 depth sensor read by the Radxa X4's onboard
# RP2040 (see ../arduino/ms5837_depth_sensor). The MCU streams CSV lines
# "depth,pressure,temperature" over USB serial; this node parses them and
# publishes steelhead_interfaces/PressureSensor on drivers/depth_sensor.
#
# This mirrors bno085_imu_publisher.py (same serial-over-USB pattern).

import rclpy
from rclpy.node import Node

from steelhead_interfaces.msg import PressureSensor

import serial


class Ms5837DepthPublisher(Node):
    def __init__(self):
        super().__init__('ms5837_depth_publisher')

        self.declare_parameter('port', '/dev/depth')
        self.declare_parameter('baud', 115200)

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value

        self.get_logger().info(f'Opening {port} at {baud} baud...')
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
        except serial.serialutil.SerialException as e:
            self.get_logger().error(f'Failed to open port {port}: {e}')
            raise

        self.pub_sensor = self.create_publisher(
            PressureSensor, 'drivers/depth_sensor', 10)

        self.timer = self.create_timer(0.02, self.timer_callback)

    def timer_callback(self):
        line = self.ser.readline().decode('utf-8', errors='ignore').strip()
        if not line:
            return

        tokens = line.split(',')
        if len(tokens) != 3:
            return

        try:
            depth = float(tokens[0])
            pressure = float(tokens[1])
            temperature = float(tokens[2])
        except ValueError:
            return  # non-numeric line (startup banner), skip silently

        msg = PressureSensor()
        msg.depth = depth              # meters
        msg.pressure = pressure        # mbar
        msg.temperature = temperature  # deg C
        self.pub_sensor.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Ms5837DepthPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
