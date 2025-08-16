#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial
import math
from collections import deque
import numpy as np


class Bno085ImuPublisher(Node):
    def __init__(self):
        super().__init__('bno085_imu_publisher')

        # Rolling average buffer size and outlier detection parameters
        self.buffer_size = 10
        self.outlier_threshold = 3.0  # standard deviations from mean
        
        # Rolling buffers for orientation (Euler angles) and acceleration
        self.yaw_buffer = deque(maxlen=self.buffer_size)
        self.pitch_buffer = deque(maxlen=self.buffer_size)
        self.roll_buffer = deque(maxlen=self.buffer_size)
        self.ax_buffer = deque(maxlen=self.buffer_size)
        self.ay_buffer = deque(maxlen=self.buffer_size)
        self.az_buffer = deque(maxlen=self.buffer_size)
        
        # Track last valid quaternion to handle wrapping
        self.last_valid_quat = None

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
    
    def is_outlier(self, value, buffer):
        """Check if a value is an outlier based on rolling average and standard deviation"""
        if len(buffer) < 3:  # Need at least 3 values for meaningful statistics
            return False
        
        buffer_array = np.array(buffer)
        mean = np.mean(buffer_array)
        std = np.std(buffer_array)
        
        if std == 0:  # All values are the same
            return abs(value - mean) > 0.1  # Small threshold for identical values
        
        z_score = abs(value - mean) / std
        return z_score > self.outlier_threshold
    
    def normalize_quaternion(self, qw, qx, qy, qz):
        """Normalize quaternion and handle sign ambiguity"""
        norm = math.sqrt(qw*qw + qx*qx + qy*qy + qz*qz)
        if norm == 0:
            return 1.0, 0.0, 0.0, 0.0
        
        qw_norm = qw / norm
        qx_norm = qx / norm
        qy_norm = qy / norm
        qz_norm = qz / norm
        
        # Handle quaternion sign ambiguity (q and -q represent same rotation)
        if self.last_valid_quat is not None:
            last_qw, last_qx, last_qy, last_qz = self.last_valid_quat
            
            # Compute dot product to check orientation
            dot = qw_norm * last_qw + qx_norm * last_qx + qy_norm * last_qy + qz_norm * last_qz
            
            # If dot product is negative, flip the quaternion
            if dot < 0:
                qw_norm = -qw_norm
                qx_norm = -qx_norm
                qy_norm = -qy_norm
                qz_norm = -qz_norm
        
        return qw_norm, qx_norm, qy_norm, qz_norm

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

                # Check for outliers in orientation data
                if (self.is_outlier(yaw_deg, self.yaw_buffer) or 
                    self.is_outlier(pitch_deg, self.pitch_buffer) or 
                    self.is_outlier(roll_deg, self.roll_buffer)):
                    # Skip this reading as it's an outlier
                    return

                # Check for outliers in acceleration data
                if (self.is_outlier(ax, self.ax_buffer) or 
                    self.is_outlier(ay, self.ay_buffer) or 
                    self.is_outlier(az, self.az_buffer)):
                    # Skip this reading as it's an outlier
                    return

                # Add values to rolling buffers after outlier check
                self.yaw_buffer.append(yaw_deg)
                self.pitch_buffer.append(pitch_deg)
                self.roll_buffer.append(roll_deg)
                self.ax_buffer.append(ax)
                self.ay_buffer.append(ay)
                self.az_buffer.append(az)

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

                # Normalize quaternion and handle wrapping
                qw, qx, qy, qz = self.normalize_quaternion(qw, qx, qy, qz)
                
                # Store this as the last valid quaternion
                self.last_valid_quat = (qw, qx, qy, qz)

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
