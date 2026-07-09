#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Wrench
from cv_bridge import CvBridge
import cv2
import numpy as np
from enum import Enum


class State(Enum):
    INITIAL_APPROACH = 'initial_approach'
    SEARCHING_RED = 'searching_red'
    SEARCHING_WHITE = 'searching_white'
    NAVIGATING = 'navigating'
    PASSING = 'passing'
    POST_PASSING = 'post_passing'
    CLEARING = 'clearing'
    COMPLETED = 'completed'


class NavNode(Node):
    def __init__(self):
        super().__init__('nav')

        self.declare_parameter('topic', '/steelhead/controls/hover_adjust')
        self.declare_parameter('image_topic', '/steelhead/drivers/front_camera/image_raw')
        self.declare_parameter('rate', 10.0)
        self.declare_parameter('force_x', 15.0)
        self.declare_parameter('gap_steer_gain', 2.0)
        self.declare_parameter('pass_lane_bias', 0.5)
        self.declare_parameter('preferred_side', 'left')
        self.declare_parameter('min_pole_area', 500.0)
        self.declare_parameter('num_gates', 3)
        self.declare_parameter('search_forward_scale', 0.01)
        self.declare_parameter('search_sweep_force', 30.0)
        self.declare_parameter('search_sweep_step', 0.2)
        self.declare_parameter('search_sweep_initial_amplitude', 1.0)
        self.declare_parameter('search_sweep_amplitude_growth', 5.0)
        self.declare_parameter('search_sweep_max_amplitude', 20.0)
        self.declare_parameter('search_sweep_min', 0.4)
        self.declare_parameter('approach_min_height', 120.0)
        self.declare_parameter('pass_confirm_frames', 5)
        self.declare_parameter('post_pass_seconds', 3.0)
        self.declare_parameter('clear_frames', 5)

        self.bridge = CvBridge()
        self.state = State.INITIAL_APPROACH
        self.lock_side = None
        self.gates_passed = 0
        self.gap_center_error = 0.0
        self.clear_count = 0
        self.lost_red_count = 0
        self.post_pass_ticks = 0
        self.initial_approach_ticks = 0
        self.tracked_white_cx = None
        self._reset_sweep()

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
        post_pass_seconds = self.get_parameter('post_pass_seconds').value
        self.initial_approach_ticks = int(post_pass_seconds * rate)
        self.timer = self.create_timer(1.0 / rate, self.publish_wrench)

        num_gates = self.get_parameter('num_gates').value
        self.get_logger().info(
            f"Nav ready: {num_gates} gates, driving forward {post_pass_seconds:.1f}s "
            f"then searching, publishing to '{topic}' at {rate} Hz, "
            f"watching '{image_topic}'"
        )

    def image_callback(self, msg: Image):
        if self.state == State.COMPLETED:
            return

        if self.state in (State.CLEARING, State.POST_PASSING, State.INITIAL_APPROACH):
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f"Failed to convert image: {e}")
            return

        closest_red = self._find_closest_red_pole(cv_image)

        if self.state == State.SEARCHING_RED:
            if closest_red is not None:
                self._lock_side_from_red(closest_red, cv_image.shape[1])
                self._reset_sweep(directed=True)
                self.get_logger().info(
                    f'Gate {self.gates_passed + 1}: red pole found; '
                    f'sweeping laterally for white pole'
                )
                self.state = State.SEARCHING_WHITE
            return

        if self.state == State.SEARCHING_WHITE:
            if closest_red is None:
                self._reset_sweep(directed=False)
                self.get_logger().info(
                    f'Gate {self.gates_passed + 1}: lost red pole; searching for red again'
                )
                self.state = State.SEARCHING_RED
                return

            partner_white = self._find_white_for_red(cv_image, closest_red)
            if partner_white is not None:
                self.get_logger().info(
                    f'Gate {self.gates_passed + 1}: white pole found; navigating through'
                )
                self.state = State.NAVIGATING
            return

        if self.state == State.NAVIGATING:
            if closest_red is None:
                self._reset_sweep()
                self.state = State.SEARCHING_RED
                self.get_logger().info(
                    f'Gate {self.gates_passed + 1}: lost red before reaching gate; resuming search'
                )
                return

            approach_min_height = self.get_parameter('approach_min_height').value
            if closest_red['height'] >= approach_min_height:
                self.state = State.PASSING
                self.lost_red_count = 0
                self.get_logger().info(
                    f'Gate {self.gates_passed + 1}: close enough; passing through'
                )

            if not self._track_gate(cv_image, closest_red):
                return
            return

        if self.state == State.PASSING:
            if closest_red is not None:
                self.lost_red_count = 0
                if not self._track_gate(cv_image, closest_red):
                    return
                return

            self.lost_red_count += 1
            confirm_frames = int(self.get_parameter('pass_confirm_frames').value)
            if self.lost_red_count < confirm_frames:
                return

            self.lost_red_count = 0
            rate = self.get_parameter('rate').value
            post_pass_seconds = self.get_parameter('post_pass_seconds').value
            self.post_pass_ticks = int(post_pass_seconds * rate)
            self.state = State.POST_PASSING
            self.get_logger().info(
                f'Gate {self.gates_passed + 1}: passed; driving forward '
                f'{post_pass_seconds:.1f}s before stopping'
            )
            return

    def _reset_sweep(self, directed=False):
        self.sweep_amplitude = self.get_parameter('search_sweep_initial_amplitude').value
        self.sweep_position = 1.0
        self.sweep_direction = -1
        self.sweep_directed = directed
        self.tracked_white_cx = None

    def _partner_lateral_sign(self):
        """Body-frame lateral sign that pans toward the partner white pole."""
        if self.lock_side == 'left':
            return -1.0
        return 1.0

    def _update_sweep(self):
        step = self.get_parameter('search_sweep_step').value
        amp_growth = self.get_parameter('search_sweep_amplitude_growth').value
        max_amp = self.get_parameter('search_sweep_max_amplitude').value
        lower = (
            self.get_parameter('search_sweep_min').value
            if self.sweep_directed else -1.0
        )

        self.sweep_position += self.sweep_direction * step

        if self.sweep_position <= lower:
            self.sweep_position = lower
            self.sweep_direction = 1
        elif self.sweep_position >= 1.0:
            self.sweep_amplitude = min(self.sweep_amplitude + amp_growth, max_amp)
            self.sweep_position = 1.0
            self.sweep_direction = -1

    def _search_lateral_force(self):
        sweep_force = self.get_parameter('search_sweep_force').value
        magnitude = self.sweep_position * self.sweep_amplitude * sweep_force
        if self.sweep_directed and self.lock_side is not None:
            return self._partner_lateral_sign() * abs(magnitude)
        return -magnitude

    def _lock_side_from_red(self, red_pole, img_w):
        if self.lock_side is not None:
            return

        preferred = self.get_parameter('preferred_side').value
        if preferred in ('left', 'right'):
            self.lock_side = preferred
        else:
            red_cx = self._pole_center_x(red_pole)
            if red_cx >= img_w / 2.0:
                self.lock_side = 'right'
            else:
                self.lock_side = 'left'
        self.get_logger().info(
            f'Locked slalom side: keep red on {self.lock_side} '
            f'(gate {self.gates_passed + 1})'
        )

    def _pole_center_x(self, pole):
        x, _, w, _ = pole['bbox']
        return x + w / 2.0

    def _find_white_for_red(self, image, red_pole):
        """Return the white pole on the passing side of the given red pole."""
        whites = self._find_all_white_poles(image)
        if not whites:
            return None

        red_cx = self._pole_center_x(red_pole)

        if self.lock_side == 'right':
            candidates = [w for w in whites if self._pole_center_x(w) < red_cx]
            if not candidates:
                return None
            partner = max(candidates, key=lambda w: self._pole_center_x(w))
        else:
            candidates = [w for w in whites if self._pole_center_x(w) > red_cx]
            if not candidates:
                return None
            partner = min(candidates, key=lambda w: self._pole_center_x(w))

        if self.tracked_white_cx is not None:
            return min(
                candidates,
                key=lambda w: abs(self._pole_center_x(w) - self.tracked_white_cx),
            )

        self.tracked_white_cx = self._pole_center_x(partner)
        return partner

    def _track_gate(self, image, red_pole):
        partner_white = self._find_white_for_red(image, red_pole)
        if partner_white is None:
            return False
        self._update_navigation_error(image, red_pole, partner_white)
        return True

    def _update_navigation_error(self, image, red_pole, white_pole):
        target_x = self._compute_target(red_pole, white_pole)
        frame_center = image.shape[1] / 2.0
        offset = (target_x - frame_center) / frame_center
        self.gap_center_error = np.clip(offset, -1.0, 1.0)

    def _compute_target(self, red_pole, white_pole):
        """Steering target between red and white pole centers (bias=0.5 is midpoint)."""
        red_cx = self._pole_center_x(red_pole)
        white_cx = self._pole_center_x(white_pole)
        bias = self.get_parameter('pass_lane_bias').value
        return red_cx + bias * (white_cx - red_cx)

    def _morph_mask(self, mask):
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        return cv2.morphologyEx(mask, cv2.MORPH_DILATE, kernel)

    def _poles_from_mask(self, mask):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        min_area = self.get_parameter('min_pole_area').value
        poles = []

        for contour in contours:
            if cv2.contourArea(contour) < min_area:
                continue

            x, y, w, h = cv2.boundingRect(contour)
            poles.append({'bbox': (x, y, w, h), 'height': h})

        return poles

    def _find_closest_red_pole(self, image):
        """Find the closest red pole in the image.

        Closest is approximated by the tallest red pole in pixel space.
        """
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower_red1 = np.array([0, 80, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 80, 70])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        poles = self._poles_from_mask(self._morph_mask(cv2.bitwise_or(mask1, mask2)))
        if not poles:
            return None
        return max(poles, key=lambda pole: pole['height'])

    def _find_all_white_poles(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        lower_white = np.array([0, 0, 150])
        upper_white = np.array([180, 70, 255])
        mask = cv2.inRange(hsv, lower_white, upper_white)
        return self._poles_from_mask(self._morph_mask(mask))

    def publish_wrench(self):
        msg = Wrench()
        msg.torque.x = 0.0
        msg.torque.y = 0.0
        msg.torque.z = 0.0

        if self.state == State.COMPLETED:
            msg.force.x = 0.0
            msg.force.y = 0.0
            msg.force.z = 0.0
        elif self.state in (State.NAVIGATING, State.PASSING, State.POST_PASSING):
            msg.force.x = self.get_parameter('force_x').value
            steer_gain = self.get_parameter('gap_steer_gain').value
            msg.force.y = -self.gap_center_error * steer_gain
            msg.force.z = 0.0
            if self.state == State.POST_PASSING:
                self.post_pass_ticks -= 1
                if self.post_pass_ticks <= 0:
                    self.gap_center_error = 0.0
                    self.clear_count = int(self.get_parameter('clear_frames').value)
                    self.state = State.CLEARING
                    self.get_logger().info(
                        f'Gate {self.gates_passed + 1}: post-pass complete; stopping'
                    )
        elif self.state == State.INITIAL_APPROACH:
            msg.force.x = self.get_parameter('force_x').value
            msg.force.y = 0.0
            msg.force.z = 0.0
            self.initial_approach_ticks -= 1
            if self.initial_approach_ticks <= 0:
                self._reset_sweep()
                self.state = State.SEARCHING_RED
                post_pass_seconds = self.get_parameter('post_pass_seconds').value
                self.get_logger().info(
                    f'Initial approach complete after {post_pass_seconds:.1f}s; '
                    f'searching for first red pole'
                )
        elif self.state == State.CLEARING:
            msg.force.x = 0.0
            msg.force.y = 0.0
            msg.force.z = 0.0
            self.clear_count -= 1
            if self.clear_count <= 0:
                self.gates_passed += 1
                num_gates = self.get_parameter('num_gates').value
                if self.gates_passed >= num_gates:
                    self.state = State.COMPLETED
                    self.get_logger().info(
                        f'Completed {self.gates_passed} gates; stopping motion'
                    )
                else:
                    self._reset_sweep()
                    self.state = State.SEARCHING_RED
                    self.get_logger().info(
                        f'Gate {self.gates_passed}/{num_gates} cleared; '
                        f'searching for next red pole'
                    )
        elif self.state in (State.SEARCHING_RED, State.SEARCHING_WHITE):
            if self.state == State.SEARCHING_RED:
                forward_scale = self.get_parameter('search_forward_scale').value
                msg.force.x = self.get_parameter('force_x').value * forward_scale
            else:
                msg.force.x = 0.0
            msg.force.y = self._search_lateral_force()
            msg.force.z = 0.0
            self._update_sweep()

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
