#!/usr/bin/env python3
"""Slalom navigator (moved into python package).
"""
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np


class SlalomNavigator(Node):
    def __init__(self):
        super().__init__('slalom_navigator')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('cmd_topic', '/cmd_vel')
        self.declare_parameter('max_linear', 0.5)
        self.declare_parameter('max_angular', 1.0)
        self.declare_parameter('hue_low', 0)
        self.declare_parameter('sat_low', 100)
        self.declare_parameter('val_low', 50)
        self.declare_parameter('hue_high', 10)
        self.declare_parameter('sat_high', 255)
        self.declare_parameter('val_high', 255)

        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        cmd_topic = self.get_parameter('cmd_topic').get_parameter_value().string_value

        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, image_topic, self.image_cb, 1)
        self.pub = self.create_publisher(Twist, cmd_topic, 1)

        self.max_linear = self.get_parameter('max_linear').get_parameter_value().double_value
        self.max_angular = self.get_parameter('max_angular').get_parameter_value().double_value

        self.hue_low = int(self.get_parameter('hue_low').get_parameter_value().integer_value)
        self.sat_low = int(self.get_parameter('sat_low').get_parameter_value().integer_value)
        self.val_low = int(self.get_parameter('val_low').get_parameter_value().integer_value)
        self.hue_high = int(self.get_parameter('hue_high').get_parameter_value().integer_value)
        self.sat_high = int(self.get_parameter('sat_high').get_parameter_value().integer_value)
        self.val_high = int(self.get_parameter('val_high').get_parameter_value().integer_value)

        self.get_logger().info('Slalom navigator started, listening to: %s' % image_topic)

    def image_cb(self, msg: Image):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error('cv_bridge error: %s' % str(e))
            return

        h, w = cv_img.shape[:2]

        hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)

        lower = np.array([self.hue_low, self.sat_low, self.val_low])
        upper = np.array([self.hue_high, self.sat_high, self.val_high])
        mask1 = cv2.inRange(hsv, lower, upper)

        if self.hue_high < self.hue_low:
            lower2 = np.array([170, self.sat_low, self.val_low])
            upper2 = np.array([180, self.sat_high, self.val_high])
            mask2 = cv2.inRange(hsv, lower2, upper2)
            mask = cv2.bitwise_or(mask1, mask2)
        else:
            mask = mask1

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        pole_centers = []
        for c in contours:
            area = cv2.contourArea(c)
            if area < 200:
                continue
            M = cv2.moments(c)
            if M['m00'] == 0:
                continue
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            pole_centers.append((cx, cy, area))

        twist = Twist()

        if len(pole_centers) == 0:
            twist.linear.x = float(self.max_linear) * 0.2
            twist.angular.z = 0.0
        elif len(pole_centers) == 1:
            cx = pole_centers[0][0]
            error_x = (cx - w / 2) / (w / 2)
            twist.linear.x = float(self.max_linear) * 0.25
            twist.angular.z = -float(self.max_angular) * error_x
        else:
            xs = sorted([p[0] for p in pole_centers])
            center_x = (xs[0] + xs[-1]) / 2.0
            error_x = (center_x - w / 2) / (w / 2)
            twist.linear.x = float(self.max_linear) * 0.5
            twist.angular.z = -float(self.max_angular) * error_x

        twist.linear.x = max(min(twist.linear.x, float(self.max_linear)), -float(self.max_linear))
        twist.angular.z = max(min(twist.angular.z, float(self.max_angular)), -float(self.max_angular))

        self.pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = SlalomNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
