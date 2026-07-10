#!/usr/bin/env python3

import math
import time

import rclpy
import serial
from rclpy.node import Node
from sensor_msgs.msg import Imu


class Bno085ImuPublisher(Node):
    RECONNECT_PERIOD = 1.0  # seconds between reconnect attempts

    def __init__(self):
        super().__init__("bno085_imu_publisher")

        self.port = "/dev/imu"
        self.baud = 115200

        self.ser = None
        self.last_connect_attempt = float("-inf")

        # A missing device at startup is not fatal: the node waits for
        # /dev/imu to (re)appear and connects from the timer
        self.get_logger().info(f"Opening {self.port} at {self.baud} baud...")
        self.try_connect()

        self.publisher_ = self.create_publisher(Imu, "/steelhead/drivers/imu/out", 10)
        # Timer: read and publish at ~100 Hz
        self.timer_ = self.create_timer(0.01, self.timer_callback)

    def try_connect(self):
        """(Re)open the serial port. Returns True on success."""
        self.last_connect_attempt = time.monotonic()
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.01)
            # Drop any backlog so we publish live data, not a stale burst
            self.ser.reset_input_buffer()
            self.get_logger().info(f"Connected to {self.port} at {self.baud} baud")
            return True
        except (serial.SerialException, OSError) as e:
            self.ser = None
            self.get_logger().warn(
                f"Failed to open {self.port}: {e} "
                f"(retrying every {self.RECONNECT_PERIOD}s)"
            )
            return False

    def disconnect(self):
        """Drop a dead connection; the timer will keep trying to reconnect."""
        if self.ser is not None:
            try:
                self.ser.close()
            except (serial.SerialException, OSError):
                pass
            self.ser = None
        self.last_connect_attempt = time.monotonic()

    def timer_callback(self):
        if self.ser is None:
            if time.monotonic() - self.last_connect_attempt < self.RECONNECT_PERIOD:
                return
            if not self.try_connect():
                return

        # Attempt to read one line from serial
        try:
            raw = self.ser.readline()
        except (serial.SerialException, OSError) as e:
            self.get_logger().error(f"Serial read failed, IMU disconnected? ({e})")
            self.disconnect()
            return

        # Unplug/replug often leaves partial or garbage bytes in the line;
        # replace undecodable bytes instead of crashing, the token checks
        # below discard the line
        line = raw.decode("utf-8", errors="replace").strip()
        if not line:
            return  # no data

        tokens = line.split(",")
        # We expect 7 tokens: status,yaw,pitch,roll,ax,ay,az
        if len(tokens) < 7:
            self.get_logger().warn(f"Not enough tokens in line: {line}")
            return

        try:
            # parse floats
            status = float(tokens[0])  # we might not use it, but let's parse
            yaw_deg = float(tokens[1])
            pitch_deg = float(tokens[2])
            roll_deg = float(tokens[3])
            ax = float(tokens[4])
            ay = float(tokens[5])
            az = float(tokens[6])

            # Convert yaw/pitch/roll in degrees to quaternion (Z->Y->X)
            yaw_rad = math.radians(yaw_deg)
            pitch_rad = math.radians(pitch_deg)
            roll_rad = math.radians(roll_deg)

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
            msg.header.frame_id = "imu_link"

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
            self.get_logger().warn(f"Failed to parse float from line: {line}")


def main(args=None):
    rclpy.init(args=args)
    node = Bno085ImuPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
