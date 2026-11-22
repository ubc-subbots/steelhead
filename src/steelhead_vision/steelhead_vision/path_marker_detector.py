#!/usr/bin/env python3

import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

from steelhead_interfaces.msg import PathMarker


class PathMarkerDetector(Node):
    """
    Finds the orange path marker in bottom camera images using a colour mask.

    Publishes a PathMarker message for every image, and a debug image showing the
    detected marker and the direction to follow while anyone is subscribed to it.
    Assumes the top of the image is the robot's front and the right of the image is
    the robot's right.
    """

    def __init__(self):
        """
        Declare parameters and set up the publishers and image subscription.
        """
        super().__init__("path_marker_detector")

        self.declare_parameter(
            "input_image_topic", "/steelhead/drivers/bottom_camera/image_raw"
        )
        self.declare_parameter("detection_topic", "/steelhead/path_marker/detection")
        self.declare_parameter(
            "debug_image_topic", "/steelhead/path_marker/debug_image"
        )
        self.declare_parameter("hsv_lower", [5, 100, 100])
        self.declare_parameter("hsv_upper", [25, 255, 255])
        self.declare_parameter("min_area_fraction", 0.01)

        input_image_topic = self.get_parameter("input_image_topic").value
        detection_topic = self.get_parameter("detection_topic").value
        debug_image_topic = self.get_parameter("debug_image_topic").value
        self.hsv_lower = np.array(self.get_parameter("hsv_lower").value, np.uint8)
        self.hsv_upper = np.array(self.get_parameter("hsv_upper").value, np.uint8)
        self.min_area_fraction = self.get_parameter("min_area_fraction").value

        self.bridge = CvBridge()
        self.kernel = np.ones((5, 5), np.uint8)

        self.detection_publisher = self.create_publisher(
            PathMarker, detection_topic, 10
        )
        self.debug_publisher = self.create_publisher(Image, debug_image_topic, 10)
        self.image_subscriber = self.create_subscription(
            Image, input_image_topic, self.image_callback, 10
        )

        self.get_logger().info(
            f"Path marker detector started (input: {input_image_topic}, "
            f"detections: {detection_topic}, debug: {debug_image_topic})"
        )

    def image_callback(self, msg):
        """
        Look for the path marker in one camera image and publish the result.

        :param msg: Bottom camera image
        """
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        detection = PathMarker()
        detection.header = msg.header

        contour = self._find_marker(image)
        if contour is not None:
            self._measure_marker(contour, image, detection)

        self.detection_publisher.publish(detection)

        if self.debug_publisher.get_subscription_count() > 0:
            debug_msg = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
            debug_msg.header = msg.header
            self.debug_publisher.publish(debug_msg)

    def _find_marker(self, image):
        """
        Find the outline of the largest orange region in the image.

        :param image: BGR camera image
        :return: The region's contour, or None if no region is big enough
        """
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE
        )
        if not contours:
            return None

        largest = max(contours, key=cv2.contourArea)
        height, width = mask.shape
        if cv2.contourArea(largest) < self.min_area_fraction * width * height:
            return None
        return largest

    def _measure_marker(self, contour, image, detection):
        """
        Fill in the detection from the marker's outline and draw it on the image.

        A straight line is fitted through the outline to get the marker's
        direction, which is flipped if needed so it points towards the top of the
        image (the robot's front).

        :param contour: Outline of the marker in the image
        :param image: BGR camera image, drawn on for the debug image
        :param detection: PathMarker message to fill in
        """
        height, width = image.shape[:2]
        vx, vy, x0, y0 = cv2.fitLine(contour, cv2.DIST_L2, 0, 0.01, 0.01).flatten()
        if vy > 0:
            vx, vy = -vx, -vy

        # Offset of the line's centre point from the image centre, in pixels
        dx = x0 - width / 2.0
        dy = y0 - height / 2.0

        detection.detected = True
        detection.heading_error = math.atan2(-vx, -vy)
        # Distance from the image centre to the line, along the line's right normal
        detection.lateral_offset = float((dx * -vy + dy * vx) / (width / 2.0))
        detection.centroid_y = float(dy / (height / 2.0))

        cv2.drawContours(image, [contour], -1, (0, 255, 0), 2)
        start = (int(x0), int(y0))
        end = (int(x0 + 100 * vx), int(y0 + 100 * vy))
        cv2.arrowedLine(image, start, end, (0, 0, 255), 3, tipLength=0.3)


def main(args=None):
    """
    Run the path marker detector node.
    """
    rclpy.init(args=args)
    node = PathMarkerDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
