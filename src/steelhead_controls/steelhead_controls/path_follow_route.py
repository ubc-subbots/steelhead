"""Path follower: WAIT -> ORIENT -> FOLLOW -> COAST -> DONE."""
import math

import rclpy
from rclpy.node import Node
from steelhead_interfaces.msg import Waypoint
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry


PASSTHROUGH = 1

FORWARD_DISTANCE = 0.5
LATERAL_GAIN = 0.3
MAX_STRAFE = 0.02

COAST_DISTANCE = 2.0

CONTROL_PERIOD = 0.05

AREA_FADE_MIN = 0.005
AREA_FADE_FULL = 0.03
FWD_FADE_CENTERED = 0.75
FWD_FADE_EDGE = 0.95

YAW_TOLERANCE = math.radians(8)
ORIENT_SAMPLES = 5

ROT_SIGN = -1


def quaternion_from_euler(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    return [
        cy * cp * cr + sy * sp * sr,
        cy * cp * sr - sy * sp * cr,
        sy * cp * sr + cy * sp * cr,
        sy * cp * cr - cy * sp * sr,
    ]


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a <= -math.pi:
        a += 2.0 * math.pi
    return a


def yaw_delta_to_align(image_angle):
    opt_a = (image_angle - math.pi / 2.0) / ROT_SIGN
    opt_b = (image_angle + math.pi / 2.0) / ROT_SIGN
    return opt_a if abs(opt_a) < abs(opt_b) else opt_b


PHASE_WAIT = 'WAIT'
PHASE_ORIENT = 'ORIENT'
PHASE_FOLLOW = 'FOLLOW'
PHASE_COAST = 'COAST'
PHASE_DONE = 'DONE'


class PathFollowRoute(Node):
    def __init__(self):
        super().__init__('path_follow_route')

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_yaw = 0.0
        self.have_state = False

        self.path_detected = False
        self.path_offset_x = 0.0
        self.path_offset_z = 0.0
        self.path_image_angle = 0.0
        self.path_area_ratio = 0.0

        self.phase = PHASE_WAIT
        self.initial_yaw = 0.0
        self.target_yaw = 0.0
        self.target_locked = False
        self.locked_z = 0.0
        self.orient_samples = []
        self.coast_start_x = 0.0
        self.coast_start_y = 0.0

        self.waypoint_pub = self.create_publisher(
            Waypoint, '/steelhead/controls/waypoint_marker/set', 10)
        self.task_complete_pub = self.create_publisher(
            Bool, '/steelhead/path_follow/task_complete', 10)

        self.create_subscription(
            Twist, '/steelhead/pathmarker/direction',
            self.pathmarker_callback, 10)
        self.create_subscription(
            Bool, '/steelhead/pathmarker/task_complete',
            self.detector_complete_callback, 10)
        self.create_subscription(
            Odometry, '/steelhead/controls/ukf/odometry/filtered',
            self.state_callback, 10)

        self.create_timer(CONTROL_PERIOD, self.control_loop)
        self.get_logger().info('Path Follow Route started')

    def pathmarker_callback(self, msg):
        if msg.linear.x > 0.5:
            self.path_detected = True
            self.path_offset_x = msg.linear.y
            self.path_offset_z = msg.linear.z
            self.path_image_angle = msg.angular.z
            self.path_area_ratio = msg.angular.x
        else:
            self.path_detected = False
            self.path_area_ratio = 0.0

    def detector_complete_callback(self, msg):
        if msg.data and self.phase == PHASE_FOLLOW:
            self.phase = PHASE_COAST
            self.coast_start_x = self.current_x
            self.coast_start_y = self.current_y
            self.get_logger().info(
                f'Marker exited view - entering COAST for {COAST_DISTANCE:.2f} m')

    def state_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_z = msg.pose.pose.position.z
        self.current_yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.have_state = True

    def control_loop(self):
        if not self.have_state or self.phase == PHASE_DONE:
            return

        if self.phase == PHASE_WAIT:
            if self.path_detected:
                self.orient_samples = []
                self.initial_yaw = self.current_yaw
                self.locked_z = self.current_z
                self.target_locked = False
                self.phase = PHASE_ORIENT
                self.get_logger().info(
                    f'PATH DETECTED - entering ORIENT '
                    f'(initial_yaw={math.degrees(self.initial_yaw):.1f} deg, '
                    f'depth_locked={self.locked_z:.2f} m)')
            return

        if self.phase == PHASE_ORIENT:
            self._run_orient()
            return

        if self.phase == PHASE_FOLLOW:
            self._run_follow()
            return

        if self.phase == PHASE_COAST:
            self._run_coast()
            return

    def _run_orient(self):
        # Hold the initial yaw while sampling; otherwise image_angle is measured
        # under a rotating camera and the average is meaningless.
        if not self.target_locked:
            if self.path_detected:
                self.orient_samples.append(self.path_image_angle)
            self._publish_hold_with_yaw(self.initial_yaw)

            if len(self.orient_samples) >= ORIENT_SAMPLES:
                mean_angle = sum(self.orient_samples) / len(self.orient_samples)
                delta = yaw_delta_to_align(mean_angle)
                self.target_yaw = wrap_angle(self.initial_yaw + delta)
                self.target_locked = True
                self.get_logger().info(
                    f'ORIENT target locked: mean_image_angle={math.degrees(mean_angle):.1f} deg, '
                    f'rotation={math.degrees(delta):+.1f} deg, '
                    f'target_yaw={math.degrees(self.target_yaw):.1f} deg')
            return

        yaw_err = wrap_angle(self.target_yaw - self.current_yaw)
        if abs(yaw_err) < YAW_TOLERANCE:
            self.phase = PHASE_FOLLOW
            self.get_logger().info(
                f'ALIGNED - entering FOLLOW (image_angle now={math.degrees(self.path_image_angle):.1f} deg)')
            return
        self._publish_hold_with_yaw(self.target_yaw)

    def _publish_hold_with_yaw(self, yaw):
        wp = self.make_waypoint(self.current_x, self.current_y, self.locked_z, yaw)
        self.waypoint_pub.publish(wp)

    def _run_follow(self):
        if not self.path_detected:
            self._publish_hold_with_yaw(self.target_yaw)
            return

        heading = self.target_yaw
        target_x = self.current_x + FORWARD_DISTANCE * math.cos(heading)
        target_y = self.current_y + FORWARD_DISTANCE * math.sin(heading)

        # Fade strafe when only the rounded cap is visible: minAreaRect's
        # lateral estimate is biased for asymmetric shapes and chasing it drifts.
        if self.path_area_ratio >= AREA_FADE_FULL:
            area_scale = 1.0
        elif self.path_area_ratio <= AREA_FADE_MIN:
            area_scale = 0.0
        else:
            area_scale = (self.path_area_ratio - AREA_FADE_MIN) / (AREA_FADE_FULL - AREA_FADE_MIN)

        fwd_abs = abs(self.path_offset_z)
        if fwd_abs <= FWD_FADE_CENTERED:
            fwd_scale = 1.0
        elif fwd_abs >= FWD_FADE_EDGE:
            fwd_scale = 0.0
        else:
            fwd_scale = (FWD_FADE_EDGE - fwd_abs) / (FWD_FADE_EDGE - FWD_FADE_CENTERED)

        gain_scale = min(area_scale, fwd_scale)

        perp = heading + math.pi / 2.0
        offset = self.path_offset_x * LATERAL_GAIN * gain_scale
        offset = max(-MAX_STRAFE, min(MAX_STRAFE, offset))
        target_x += offset * math.cos(perp)
        target_y += offset * math.sin(perp)

        wp = self.make_waypoint(target_x, target_y, self.locked_z, heading)
        self.waypoint_pub.publish(wp)

        if abs(self.path_offset_x) > 0.1 and gain_scale > 0.0:
            direction = 'RIGHT' if self.path_offset_x > 0 else 'LEFT'
            self.get_logger().info(
                f'Correcting {direction}: offset={self.path_offset_x:.2f}, '
                f'area={self.path_area_ratio:.3f}, fwd={self.path_offset_z:+.2f}, '
                f'gain_scale={gain_scale:.2f}')
        elif gain_scale == 0.0:
            self.get_logger().info(
                f'Coasting straight (area={self.path_area_ratio:.3f}, fwd={self.path_offset_z:+.2f})')

    def _run_coast(self):
        traveled = math.hypot(
            self.current_x - self.coast_start_x,
            self.current_y - self.coast_start_y)
        if traveled >= COAST_DISTANCE:
            self.phase = PHASE_DONE
            self.get_logger().info(
                f'COAST complete ({traveled:.2f} m traveled) - PATH FOLLOWING COMPLETE')
            done = Bool()
            done.data = True
            self.task_complete_pub.publish(done)
            return

        heading = self.target_yaw
        target_x = self.current_x + FORWARD_DISTANCE * math.cos(heading)
        target_y = self.current_y + FORWARD_DISTANCE * math.sin(heading)
        wp = self.make_waypoint(target_x, target_y, self.locked_z, heading)
        self.waypoint_pub.publish(wp)

    def make_waypoint(self, x, y, z, yaw):
        wp = Waypoint()
        wp.pose.position.x = x
        wp.pose.position.y = y
        wp.pose.position.z = z

        q = quaternion_from_euler(0.0, 0.0, yaw)
        wp.pose.orientation.w = q[0]
        wp.pose.orientation.x = q[1]
        wp.pose.orientation.y = q[2]
        wp.pose.orientation.z = q[3]

        wp.distance.position.x = 0.2
        wp.distance.position.y = 0.2
        wp.distance.position.z = 0.3

        # waypoint_marker decomposes this via tf2 getRPY; a quat at gimbal lock
        # (e.g. RPY 1.57,1.57,1.57) silently produces ~pi/2 yaw tolerance and
        # waypoints are marked achieved instantly, zeroing the PID error.
        dist_q = quaternion_from_euler(math.radians(30), math.radians(30), math.radians(5))
        wp.distance.orientation.w = dist_q[0]
        wp.distance.orientation.x = dist_q[1]
        wp.distance.orientation.y = dist_q[2]
        wp.distance.orientation.z = dist_q[3]

        wp.success = False
        wp.type = PASSTHROUGH
        wp.duration = 0.0
        return wp


def main(args=None):
    rclpy.init(args=args)
    node = PathFollowRoute()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
