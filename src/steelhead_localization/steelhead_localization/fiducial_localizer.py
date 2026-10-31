from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Imu

from steelhead_interfaces.msg import DetectionBoxArray
from steelhead_localization.fiducial_geometry import (
    box_to_optical_point,
    detection_weight,
    fuse_estimates,
    optical_to_body,
    robot_position_from_symbol,
)
from steelhead_localization.symbols import load_symbols, unique_by_label


UNMEASURED_VARIANCE = 1e3
NANOSECONDS_PER_SECOND = 1e9


class FiducialLocalizer(Node):

    def __init__(self):
        super().__init__("fiducial_localizer")

        self.declare_parameter("symbols_file", "")
        self.declare_parameter("camera_offset", [0.45, -0.06, 0.0])
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("min_confidence", 0.3)
        self.declare_parameter("range_noise_ratio", 0.05)
        self.declare_parameter("max_jump", 1.0)
        self.declare_parameter("jump_reset_time", 3.0)
        self.declare_parameter("max_imu_age", 0.5)
        self.declare_parameter(
            "detections_topic", "/steelhead/yolo_detector/front/detections"
        )
        self.declare_parameter(
            "camera_info_topic", "/steelhead/drivers/front_camera/camera_info"
        )
        self.declare_parameter("imu_topic", "/steelhead/drivers/imu/out")
        self.declare_parameter("pose_topic", "/steelhead/localization/fiducial/pose")

        symbols_file = self.get_parameter("symbols_file").value
        if not symbols_file:
            raise ValueError("the 'symbols_file' parameter must be set")
        self.symbols_ = unique_by_label(load_symbols(symbols_file))

        self.camera_offset_ = np.array(self.get_parameter("camera_offset").value)
        self.frame_id_ = self.get_parameter("frame_id").value
        self.min_confidence_ = self.get_parameter("min_confidence").value
        self.range_noise_ratio_ = self.get_parameter("range_noise_ratio").value
        self.max_jump_ = self.get_parameter("max_jump").value
        self.jump_reset_ns_ = (
            self.get_parameter("jump_reset_time").value * NANOSECONDS_PER_SECOND
        )
        self.max_imu_age_ns_ = (
            self.get_parameter("max_imu_age").value * NANOSECONDS_PER_SECOND
        )

        self.intrinsics_ = None
        self.image_size_ = None
        self.imu_buffer_ = deque(maxlen=400)
        self.last_position_ = None
        self.last_stamp_ns_ = None

        self.pose_pub_ = self.create_publisher(
            PoseWithCovarianceStamped, self.get_parameter("pose_topic").value, 10
        )
        self.create_subscription(
            CameraInfo,
            self.get_parameter("camera_info_topic").value,
            self._camera_info_callback,
            10,
        )
        self.create_subscription(
            Imu, self.get_parameter("imu_topic").value, self._imu_callback, 50
        )
        self.create_subscription(
            DetectionBoxArray,
            self.get_parameter("detections_topic").value,
            self._detections_callback,
            10,
        )

        self.get_logger().info(
            f"Fiducial localizer started with {len(self.symbols_)} usable symbols: "
            f"{', '.join(sorted(self.symbols_))}"
        )

    def _camera_info_callback(self, msg):
        self.intrinsics_ = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])
        self.image_size_ = (msg.width, msg.height) if msg.width else None

    def _imu_callback(self, msg):
        q = msg.orientation
        self.imu_buffer_.append(
            (Time.from_msg(msg.header.stamp).nanoseconds, (q.x, q.y, q.z, q.w))
        )

    def _detections_callback(self, msg):
        if self.intrinsics_ is None:
            self.get_logger().warning(
                "Waiting for camera info", throttle_duration_sec=5.0
            )
            return

        stamp_ns = Time.from_msg(msg.header.stamp).nanoseconds
        orientation = self._orientation_at(stamp_ns)
        if orientation is None:
            self.get_logger().warning(
                "No IMU reading near the detection time", throttle_duration_sec=5.0
            )
            return

        positions = []
        weights = []
        depths = []
        for box in msg.boxes:
            symbol = self.symbols_.get(box.label)
            if symbol is None or box.confidence < self.min_confidence_:
                continue
            optical = box_to_optical_point(
                (box.x, box.y, box.width, box.height),
                self.intrinsics_,
                symbol.height,
                self.image_size_,
            )
            if optical is None:
                continue
            body = optical_to_body(optical, self.camera_offset_)
            positions.append(
                robot_position_from_symbol(symbol.position, body, orientation)
            )
            weights.append(detection_weight(box.confidence, optical[2]))
            depths.append(optical[2])

        position = fuse_estimates(positions, weights)
        if position is None:
            return
        if self._is_jump(position, stamp_ns):
            self.get_logger().debug(f"Rejected jump to {np.round(position, 2)}")
            return

        self.last_position_ = position
        self.last_stamp_ns_ = stamp_ns
        depth = np.average(depths, weights=weights)
        self._publish(msg.header.stamp, position, orientation, depth)

    def _orientation_at(self, stamp_ns):
        if not self.imu_buffer_:
            return None
        imu_stamp_ns, orientation = min(
            self.imu_buffer_, key=lambda reading: abs(reading[0] - stamp_ns)
        )
        if abs(imu_stamp_ns - stamp_ns) > self.max_imu_age_ns_:
            return None
        return orientation

    def _is_jump(self, position, stamp_ns):
        if self.last_position_ is None:
            return False
        if stamp_ns - self.last_stamp_ns_ > self.jump_reset_ns_:
            return False
        return np.linalg.norm(position - self.last_position_) > self.max_jump_

    def _publish(self, stamp, position, orientation, depth):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id_
        msg.pose.pose.position.x = float(position[0])
        msg.pose.pose.position.y = float(position[1])
        msg.pose.pose.position.z = float(position[2])
        (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        ) = orientation

        position_variance = float((self.range_noise_ratio_ * depth) ** 2)
        covariance = np.diag([position_variance] * 3 + [UNMEASURED_VARIANCE] * 3)
        msg.pose.covariance = covariance.flatten().tolist()
        self.pose_pub_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    fiducial_localizer = FiducialLocalizer()
    try:
        rclpy.spin(fiducial_localizer)
    except KeyboardInterrupt:
        pass  # To force exit code 0
    fiducial_localizer.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
