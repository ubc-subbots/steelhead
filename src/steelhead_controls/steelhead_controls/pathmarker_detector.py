#!/usr/bin/env python3
"""
Orange pathmarker detector for the down camera.

Publishes /steelhead/pathmarker/direction (Twist):
  linear.x  : 1.0 if detected, 0.0 if not
  linear.y  : normalized lateral offset (-1..1, +ve = right of image center)
  linear.z  : normalized forward offset (-1..1, +ve = below image center)
  angular.x : fraction of image area covered by the contour
  angular.z : long-axis angle in radians (-pi/2..pi/2)
Plus /steelhead/pathmarker/debug (annotated Image) and
/steelhead/pathmarker/task_complete (Bool, fires after the marker exits view).
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np
import math


class PathmarkerDetector(Node):
    def __init__(self):
        super().__init__('pathmarker_detector')

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/steelhead/drivers/down_camera/image_raw',
            self.image_callback,
            10
        )

        self.direction_pub = self.create_publisher(
            Twist,
            '/steelhead/pathmarker/direction',
            10
        )

        self.debug_pub = self.create_publisher(
            Image,
            '/steelhead/pathmarker/debug',
            10
        )

        self.task_complete_pub = self.create_publisher(
            Bool,
            '/steelhead/pathmarker/task_complete',
            10
        )

        # HSV range; tune for underwater lighting if needed.
        self.orange_lower = np.array([5, 100, 100])
        self.orange_upper = np.array([25, 255, 255])

        # Real marker is 1.2m x 0.15m (ratio ~8); loosen to accept partial views.
        self.min_aspect_ratio = 3.0
        self.max_aspect_ratio = 20.0

        self.min_contour_area = 300  # pixels

        self.was_detected = False
        self.frames_without_detection = 0
        self.frames_to_confirm_exit = 15  # ~0.5s at 30 fps

        self.get_logger().info('Pathmarker Detector started')

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'CV Bridge error: {e}')
            return

        detected, angle, center, area_ratio, debug_image = self.detect_pathmarker(cv_image)

        height, width = cv_image.shape[:2]
        direction_msg = Twist()

        if detected:
            self.was_detected = True
            self.frames_without_detection = 0

            direction_msg.linear.x = 1.0
            direction_msg.linear.y = (center[0] - width / 2) / (width / 2)
            direction_msg.linear.z = (center[1] - height / 2) / (height / 2)
            direction_msg.angular.z = angle
            direction_msg.angular.x = area_ratio
        else:
            direction_msg.linear.x = 0.0

            if self.was_detected:
                self.frames_without_detection += 1

                if self.frames_without_detection >= self.frames_to_confirm_exit:
                    self.get_logger().info('PATH TASK COMPLETE - pathmarker exited view')
                    task_complete_msg = Bool()
                    task_complete_msg.data = True
                    self.task_complete_pub.publish(task_complete_msg)

                    self.was_detected = False
                    self.frames_without_detection = 0

        self.direction_pub.publish(direction_msg)

        try:
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, 'bgr8')
            self.debug_pub.publish(debug_msg)
        except Exception as e:
            self.get_logger().error(f'Debug publish error: {e}')

    def detect_pathmarker(self, image):
        debug_image = image.copy()
        height, width = image.shape[:2]

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self.orange_lower, self.orange_upper)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        mask_colored = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        mask_colored[:, :, 0] = 0
        mask_colored[:, :, 1] = 0
        debug_image = cv2.addWeighted(debug_image, 0.7, mask_colored, 0.3, 0)

        cv2.line(debug_image, (width // 2 - 20, height // 2), (width // 2 + 20, height // 2), (255, 255, 255), 1)
        cv2.line(debug_image, (width // 2, height // 2 - 20), (width // 2, height // 2 + 20), (255, 255, 255), 1)

        best_contour = None
        best_rect = None
        best_score = 0

        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self.min_contour_area:
                continue

            rect = cv2.minAreaRect(contour)
            (cx, cy), (w, h), angle = rect

            if w < h:
                w, h = h, w
                angle += 90

            if h == 0:
                continue

            aspect_ratio = w / h

            if self.min_aspect_ratio <= aspect_ratio <= self.max_aspect_ratio:
                score = area
                if score > best_score:
                    best_score = score
                    best_contour = contour
                    best_rect = rect

        if best_contour is not None and best_rect is not None:
            box = cv2.boxPoints(best_rect)
            box = np.int0(box)
            cv2.drawContours(debug_image, [box], 0, (0, 255, 0), 3)

            (cx, cy), (w, h), angle = best_rect

            if w < h:
                w, h = h, w
                angle += 90

            while angle > 90:
                angle -= 180
            while angle < -90:
                angle += 180

            angle_rad = math.radians(angle)
            area_ratio = cv2.contourArea(best_contour) / (width * height)

            arrow_len = 60
            end_x = int(cx + arrow_len * math.cos(angle_rad))
            end_y = int(cy + arrow_len * math.sin(angle_rad))
            cv2.arrowedLine(debug_image, (int(cx), int(cy)), (end_x, end_y), (0, 0, 255), 3)

            cv2.line(debug_image, (width // 2, height // 2), (int(cx), int(cy)), (255, 255, 0), 2)
            cv2.circle(debug_image, (int(cx), int(cy)), 8, (0, 255, 0), -1)

            cv2.putText(debug_image, 'PATH DETECTED', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(debug_image, f'Angle: {angle:.1f} deg', (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(debug_image, f'Offset: ({(cx - width/2):.0f}, {(cy - height/2):.0f})', (10, 90),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            return True, angle_rad, (cx, cy), area_ratio, debug_image

        cv2.putText(debug_image, 'NO PATH', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        if self.was_detected:
            cv2.putText(debug_image, f'Lost for {self.frames_without_detection} frames', (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)

        return False, 0.0, (0, 0), 0.0, debug_image


def main(args=None):
    rclpy.init(args=args)
    node = PathmarkerDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
