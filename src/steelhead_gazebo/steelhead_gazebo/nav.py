#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Wrench
from cv_bridge import CvBridge
import cv2
import numpy as np


class NavNode(Node):
    def __init__(self):
        super().__init__('nav')

        self.declare_parameter('topic', '/steelhead/controls/hover_adjust')
        self.declare_parameter('image_topic', '/steelhead/drivers/front_camera/image_raw')
        self.declare_parameter('rate', 10.0)
        self.declare_parameter('force_x', 15.0)
        self.declare_parameter('force_y', 0.0)
        self.declare_parameter('force_z', 0.0)
        self.declare_parameter('torque_x', 0.0)
        self.declare_parameter('torque_y', 0.0)
        self.declare_parameter('torque_z', 0.0)
        self.declare_parameter('min_pole_area', 500.0)

        self.bridge = CvBridge()
        self.pole_seen = False

        topic = self.get_parameter('topic').value
        self.publisher_ = self.create_publisher(Wrench, topic, 10)

        image_topic = self.get_parameter('image_topic').value
        self.subscription_ = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            1
        )

        rate = self.get_parameter('rate').value
        self.timer = self.create_timer(1.0 / rate, self.publish_wrench)

        self.get_logger().info(
            f"Publishing constant wrench to '{topic}' at {rate} Hz and watching '{image_topic}' for red poles"
        )

    def image_callback(self, msg: Image):
        if self.pole_seen:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f"Failed to convert image: {e}")
            return

        if self._detect_red_pole(cv_image):
            self.pole_seen = True
            self.get_logger().info('Red pole detected: stopping forward motion')

        if self._detect_white_pole(cv_image):
            self.pole_seen = True
            self.get_logger().info('white pole detected: stopping forward motion')

    def _detect_red_pole(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower_red1 = np.array([0, 50, 50])
        upper_red1 = np.array([15, 255, 255])
        lower_red2 = np.array([160, 50, 50])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        min_area = self.get_parameter('min_pole_area').value

        for contour in contours:
            if cv2.contourArea(contour) >= min_area:
                return True
        return False

    def _detect_white_pole(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower_white1 = np.array([0, 50, 50])
        upper_white1 = np.array([15, 255, 255])
        lower_white2 = np.array([160, 50, 50])
        upper_white2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_white1, upper_white1)
        mask2 = cv2.inRange(hsv, lower_white2, upper_white2)
        mask = cv2.bitwise_or(mask1, mask2)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        min_area = self.get_parameter('min_pole_area').value

        for contour in contours:
            if cv2.contourArea(contour) >= min_area:
                return True
        return False

    def _find_center(self, redImage, whiteImage):
        """Return the pixel center between the red and white pole detections.

        Args:
            redImage: BGR image containing the red pole.
            whiteImage: BGR image containing the white pole.

        Returns:
            The x coordinate of the gap center as a float, or None if the gap
            cannot be computed.
        """
        def extract_centroid(image, color='red'):
            if image is None:
                return None

            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            if color == 'red':
                lower1 = np.array([0, 50, 50])
                upper1 = np.array([15, 255, 255])
                lower2 = np.array([160, 50, 50])
                upper2 = np.array([180, 255, 255])
                mask1 = cv2.inRange(hsv, lower1, upper1)
                mask2 = cv2.inRange(hsv, lower2, upper2)
                mask = cv2.bitwise_or(mask1, mask2)
            else:
                lower = np.array([0, 0, 180])
                upper = np.array([180, 40, 255])
                mask = cv2.inRange(hsv, lower, upper)

            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, kernel)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            min_area = self.get_parameter('min_pole_area').value
            best_contour = None
            best_area = 0
            for contour in contours:
                area = cv2.contourArea(contour)
                if area < min_area:
                    continue
                if area > best_area:
                    best_area = area
                    best_contour = contour

            if best_contour is None:
                return None

            moments = cv2.moments(best_contour)
            if moments['m00'] == 0:
                return None
            return float(moments['m10'] / moments['m00'])

        red_cx = extract_centroid(redImage, color='red')
        white_cx = extract_centroid(whiteImage, color='white')
        if red_cx is None or white_cx is None:
            return None

        return (red_cx + white_cx) / 2.0

    def publish_wrench(self):
        msg = Wrench()
        if self.pole_seen:
            msg.force.x = 0.0
            msg.force.y = 0.0
            msg.force.z = 0.0
        else:
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
