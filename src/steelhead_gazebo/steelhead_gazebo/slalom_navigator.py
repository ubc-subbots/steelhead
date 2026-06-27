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
    SEARCHING   = 'searching'
    APPROACHING = 'approaching'
    PASSING     = 'passing'
    CLEAR       = 'clear'
    COMPLETED   = 'completed'


class SlalomNavigator(Node):
    def __init__(self):
        super().__init__('slalom_navigator')

        # Topics
        self.declare_parameter('image_topic', '/steelhead/drivers/front_camera/image_raw')
        self.declare_parameter('wrench_topic', '/steelhead/controls/input_forces')

        # Motion
        self.declare_parameter('forward_force', 15.0)
        self.declare_parameter('search_forward_scale', 0.5)
        self.declare_parameter('lateral_gain', 1.5)
        self.declare_parameter('passing_lateral_scale', 1.5)
        self.declare_parameter('control_rate', 20.0)
        # Constant upward force to counter negative buoyancy (same as holding 'q' in teleop)
        self.declare_parameter('buoyancy_trim_z', 10.0)

        # Task
        self.declare_parameter('num_gates', 3)
        self.declare_parameter('auto_lock_side', True)
        self.declare_parameter('keep_side', 'right')
        self.declare_parameter('approach_area', 2500)
        self.declare_parameter('clear_frames', 15)
        self.declare_parameter('min_contour_area', 200)

        # Optional depth trim via force.z when navigating a visible gate
        self.declare_parameter('enable_depth_correction', False)
        self.declare_parameter('target_pole_y_fraction', 0.45)
        self.declare_parameter('depth_gain', 8.0)

        # Red HSV
        self.declare_parameter('r_h_low', 0)
        self.declare_parameter('r_s_low', 100)
        self.declare_parameter('r_v_low', 50)
        self.declare_parameter('r_h_high', 10)
        self.declare_parameter('r_s_high', 255)
        self.declare_parameter('r_v_high', 255)

        # White HSV
        self.declare_parameter('w_h_low', 0)
        self.declare_parameter('w_s_low', 0)
        self.declare_parameter('w_v_low', 200)
        self.declare_parameter('w_h_high', 180)
        self.declare_parameter('w_s_high', 70)
        self.declare_parameter('w_v_high', 255)

        self.bridge = CvBridge()

        self.sub = self.create_subscription(
            Image,
            self.get_parameter('image_topic').value,
            self.image_cb,
            1
        )

        self.wrench_pub = self.create_publisher(
            Wrench,
            self.get_parameter('wrench_topic').value,
            10
        )

        self.state = State.SEARCHING
        self.clear_count = 0
        self.lock_side = None
        self.gates_passed = 0
        self.latest_gate = None
        self.img_w = 640
        self.img_h = 480

        rate = self.get_parameter('control_rate').value
        self.control_timer = self.create_timer(1.0 / rate, self.control_loop)

        self.get_logger().info(
            f"Slalom navigator ready (target {self.get_parameter('num_gates').value} gate sets, "
            f"buoyancy trim z={self.get_parameter('buoyancy_trim_z').value})"
        )

    def lateral_force(self, err, scale=1.0):
        forward_force = self.get_parameter('forward_force').value
        raw = -self.get_parameter('lateral_gain').value * scale * err * forward_force
        return max(-forward_force, min(forward_force, raw))

    def depth_force(self, gate, img_h):
        if not self.get_parameter('enable_depth_correction').value:
            return 0.0

        red, lw, rw = gate
        channel_cy = (red[1] + lw[1] + rw[1]) / 3.0
        target_y = img_h * self.get_parameter('target_pole_y_fraction').value
        err = (channel_cy - target_y) / (img_h / 2.0)

        forward_force = self.get_parameter('forward_force').value
        raw = err * self.get_parameter('depth_gain').value
        return max(-forward_force, min(forward_force, raw))

    def detect(self, hsv):
        r_low = np.array([
            self.get_parameter('r_h_low').value,
            self.get_parameter('r_s_low').value,
            self.get_parameter('r_v_low').value,
        ])
        r_high = np.array([
            self.get_parameter('r_h_high').value,
            self.get_parameter('r_s_high').value,
            self.get_parameter('r_v_high').value,
        ])

        mask_r1 = cv2.inRange(hsv, r_low, r_high)
        mask_r2 = cv2.inRange(hsv, np.array([170, 100, 50]), np.array([180, 255, 255]))
        mask_r = cv2.bitwise_or(mask_r1, mask_r2)

        w_low = np.array([
            self.get_parameter('w_h_low').value,
            self.get_parameter('w_s_low').value,
            self.get_parameter('w_v_low').value,
        ])
        w_high = np.array([
            self.get_parameter('w_h_high').value,
            self.get_parameter('w_s_high').value,
            self.get_parameter('w_v_high').value,
        ])
        mask_w = cv2.inRange(hsv, w_low, w_high)

        min_area = self.get_parameter('min_contour_area').value

        def extract(mask):
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_DILATE, kernel)

            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            out = []
            for c in cnts:
                a = cv2.contourArea(c)
                if a < min_area:
                    continue
                m = cv2.moments(c)
                if m["m00"] == 0:
                    continue
                cx = int(m["m10"] / m["m00"])
                cy = int(m["m01"] / m["m00"])
                out.append((cx, cy, a))
            return out

        return extract(mask_r), extract(mask_w)

    def get_gate(self, reds, whites):
        if len(reds) == 0 or len(whites) < 2:
            return None

        red = max(reds, key=lambda x: x[2])

        left_whites = [w for w in whites if w[0] < red[0]]
        right_whites = [w for w in whites if w[0] > red[0]]

        if len(left_whites) == 0 or len(right_whites) == 0:
            return None

        left_white = max(left_whites, key=lambda x: x[0])
        right_white = min(right_whites, key=lambda x: x[0])

        return red, left_white, right_white

    def lock_side_from_gate(self, gate, img_w):
        if self.lock_side is not None:
            return

        red_cx = gate[0][0]
        if self.get_parameter('auto_lock_side').value:
            if red_cx >= img_w / 2.0:
                self.lock_side = 'right'
            else:
                self.lock_side = 'left'
            self.get_logger().info(
                f"Auto-locked slalom side: keep RED on {self.lock_side} "
                f"(gate set {self.gates_passed + 1})"
            )
        else:
            self.lock_side = self.get_parameter('keep_side').value
            self.get_logger().info(
                f"Using configured slalom side: keep RED on {self.lock_side}"
            )

    def compute_target(self, gate, img_w):
        red, lw, rw = gate
        self.lock_side_from_gate(gate, img_w)

        if self.lock_side == 'right':
            target_x = (lw[0] + red[0]) / 2.0
        else:
            target_x = (rw[0] + red[0]) / 2.0

        return (target_x - img_w / 2.0) / (img_w / 2.0)

    def apply_navigation(self, wrench, gate, forward_scale=1.0, lateral_scale=1.0):
        forward_force = self.get_parameter('forward_force').value
        err = self.compute_target(gate, self.img_w)
        wrench.force.x = forward_force * forward_scale
        wrench.force.y = self.lateral_force(err, scale=lateral_scale)
        wrench.force.z = self.depth_force(gate, self.img_h)

    def update_state(self, gate):
        forward_force = self.get_parameter('forward_force').value

        if self.state == State.SEARCHING:
            if gate:
                self.state = State.APPROACHING
                self.get_logger().info(f"Gate set {self.gates_passed + 1}: searching → approaching")

        elif self.state == State.APPROACHING:
            if not gate:
                self.state = State.SEARCHING
            elif gate[0][2] > self.get_parameter('approach_area').value:
                self.state = State.PASSING
                self.get_logger().info(f"Gate set {self.gates_passed + 1}: approaching → passing")

        elif self.state == State.PASSING:
            if not gate:
                self.clear_count = self.get_parameter('clear_frames').value
                self.state = State.CLEAR
                self.get_logger().info(f"Gate set {self.gates_passed + 1}: passing → clear")

        elif self.state == State.CLEAR:
            self.clear_count -= 1
            if self.clear_count <= 0:
                self.gates_passed += 1
                num_gates = self.get_parameter('num_gates').value
                if self.gates_passed >= num_gates:
                    self.state = State.COMPLETED
                    self.get_logger().info(
                        f"Slalom task complete: navigated {self.gates_passed} gate sets"
                    )
                else:
                    self.state = State.SEARCHING
                    self.get_logger().info(
                        f"Cleared gate set {self.gates_passed}/{num_gates}, searching for next set"
                    )

    def build_wrench(self):
        wrench = Wrench()
        gate = self.latest_gate
        forward_force = self.get_parameter('forward_force').value
        search_scale = self.get_parameter('search_forward_scale').value

        if self.state == State.COMPLETED:
            return wrench

        if self.state == State.SEARCHING:
            wrench.force.x = forward_force * search_scale

        elif self.state == State.APPROACHING:
            if gate:
                self.apply_navigation(wrench, gate)
            else:
                wrench.force.x = forward_force * search_scale

        elif self.state == State.PASSING:
            if gate:
                self.apply_navigation(
                    wrench, gate,
                    lateral_scale=self.get_parameter('passing_lateral_scale').value,
                )
            else:
                wrench.force.x = forward_force * search_scale

        elif self.state == State.CLEAR:
            wrench.force.x = forward_force * search_scale

        return wrench

    def control_loop(self):
        wrench = self.build_wrench()
        if self.state != State.COMPLETED:
            wrench.force.z += self.get_parameter('buoyancy_trim_z').value
        self.wrench_pub.publish(wrench)

    def image_cb(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.img_h, self.img_w = img.shape[:2]
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        reds, whites = self.detect(hsv)
        gate = self.get_gate(reds, whites)
        self.latest_gate = gate

        if self.state != State.COMPLETED:
            self.update_state(gate)


def main():
    rclpy.init()
    node = SlalomNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
