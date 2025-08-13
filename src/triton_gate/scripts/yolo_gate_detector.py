#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from triton_interfaces.msg import ObjectOffset
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped
import cv2
import numpy as np
from ultralytics import YOLO
import tf_transformations
import math
from cv_bridge import CvBridge

class YOLOGateDetector(Node):
    def __init__(self):
        super().__init__('yolo_gate_detector')
        
        # Declare parameters
        self.declare_parameter('model_path', 'arvp_front.pt')
        self.declare_parameter('debug', False)
        self.declare_parameter('confidence_threshold', 0.5)
        
        # Get parameters
        self.model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.debug = self.get_parameter('debug').get_parameter_value().bool_value
        self.confidence_threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        
        if self.debug:
            self.get_logger().info('YOLO Gate Detector DEBUG ON')
        else:
            self.get_logger().info('YOLO Gate Detector DEBUG OFF')
        
        # Initialize YOLO model
        try:
            self.model = YOLO(self.model_path)
            self.get_logger().info(f'Loaded YOLO model from {self.model_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to load YOLO model: {e}')
            return
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Subscribers
        self.image_subscription = self.create_subscription(
            Image,
            '/triton/drivers/front_camera/image_raw',
            self.image_callback,
            10
        )
        
        # Publishers - matching the C++ node's publishers
        self.gate_pose_publisher = self.create_publisher(
            ObjectOffset,
            'detector/gate_pose',
            10
        )
        
        if self.debug:
            self.gate_pose_only_publisher = self.create_publisher(
                PoseStamped,
                'detector/gate_pose_only',
                10
            )
            self.debug_image_publisher = self.create_publisher(
                Image,
                'detector/debug/yolo_detection',
                10
            )
        
        self.gate_offset_publisher = self.create_publisher(
            Float32MultiArray,
            'detector/gate_offset',
            10
        )
        
        # Gate detection constants (matching C++ implementation)
        self.standard_pixel_width = 550.0
        self.standard_distance = 1.0  # in meters
        self.standard_width = 1.5  # in meters
        self.focal_length = (self.standard_pixel_width * self.standard_distance) / self.standard_width
        
        self.get_logger().info('YOLO Gate Detector successfully started!')
    
    def image_callback(self, msg):
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Run YOLO detection
            results = self.model(cv_image)[0]
            detections = results.boxes
            
            if detections is not None and len(detections) > 0:
                # Get detection data
                detection_locations = detections.xywh  # center_x, center_y, width, height
                detection_confidence = detections.conf
                detection_ids = detections.cls
                
                # Find Full Gate detections (class 2)
                gate_detections = []
                for i in range(len(detection_ids)):
                    if detection_ids[i] == 2 and detection_confidence[i] >= self.confidence_threshold:  # Full Gate class
                        gate_detections.append({
                            'bbox': detection_locations[i],
                            'confidence': detection_confidence[i].item(),
                            'class_id': detection_ids[i].item()
                        })
                 
                if gate_detections:
                    # Use the detection with highest confidence
                    best_detection = max(gate_detections, key=lambda x: x['confidence'])
                    self.process_gate_detection(best_detection, cv_image)
                    
                    self.get_logger().info(f'Detected gate with confidence: {best_detection["confidence"]:.3f}')
                else:
                    self.get_logger().debug('No gate detections above threshold')
            else:
                self.get_logger().debug('No detections found')
                
        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')
    
    def process_gate_detection(self, detection, cv_image):
        # Extract YOLO detection data (xywh format: center_x, center_y, width, height)
        center_x, center_y, width, height = detection['bbox']
        confidence = detection['confidence']
        
        # Convert to numpy for calculations
        center_x = center_x.item()
        center_y = center_y.item()
        width = width.item()
        height = height.item()
        
        # Get image dimensions
        img_height, img_width = cv_image.shape[:2]
        
        # Convert YOLO format to corner coordinates (matching C++ implementation expectations)
        top_left_x = center_x - width / 2
        top_left_y = center_y - height / 2
        bot_right_x = center_x + width / 2
        bot_right_y = center_y + height / 2
        
        # Calculate distance using the same method as C++ implementation
        curr_pixel_width = width
        distance = (self.standard_width * self.focal_length) / curr_pixel_width
        
        # Calculate center point of frame
        frame_center_x = img_width / 2
        frame_center_y = img_height / 2
        
        # Calculate offset from frame center to gate center
        offset_x = center_x - frame_center_x
        offset_y = center_y - frame_center_y
        
        # Calculate 3D distances (matching C++ math)
        distance_x = offset_x / curr_pixel_width * distance
        distance_y = offset_y / curr_pixel_width * distance
        
        # Create pose message (matching C++ implementation)
        gate_pose = ObjectOffset()
        gate_pose.class_id = 2  # Full Gate class ID
        gate_pose.pose.position.x = distance
        gate_pose.pose.position.y = -distance_x  # ENU
        gate_pose.pose.position.z = -distance_y  # ENU, y=0 on image is top
        
        # Set orientation (assuming head-on for now, matching C++ implementation)
        gate_yaw = 0.001
        quaternion = tf_transformations.quaternion_from_euler(0.001, 0.001, gate_yaw)
        gate_pose.pose.orientation.x = quaternion[0]
        gate_pose.pose.orientation.y = quaternion[1]
        gate_pose.pose.orientation.z = quaternion[2]
        gate_pose.pose.orientation.w = quaternion[3]
        
        # Publish gate pose
        self.gate_pose_publisher.publish(gate_pose)
        
        # Publish offset
        offset_msg = Float32MultiArray()
        offset_msg.data = [float(offset_x), float(offset_y)]
        self.gate_offset_publisher.publish(offset_msg)
        
        if self.debug:
            # Publish pose only for debug
            pose_only_msg = PoseStamped()
            pose_only_msg.pose = gate_pose.pose
            pose_only_msg.header.stamp = self.get_clock().now().to_msg()
            pose_only_msg.header.frame_id = 'map'
            self.gate_pose_only_publisher.publish(pose_only_msg)
            
            # Create debug visualization
            debug_image = cv_image.copy()
            
            # Draw bounding box
            cv2.rectangle(debug_image, 
                         (int(top_left_x), int(top_left_y)), 
                         (int(bot_right_x), int(bot_right_y)), 
                         (0, 255, 0), 2)
            
            # Draw center points
            cv2.circle(debug_image, (int(center_x), int(center_y)), 12, (0, 255, 0), 6)
            cv2.circle(debug_image, (int(frame_center_x), int(frame_center_y)), 12, (0, 0, 255), 6)
            
            # Draw line from frame center to gate center
            cv2.line(debug_image, 
                    (int(frame_center_x), int(frame_center_y)), 
                    (int(center_x), int(center_y)), 
                    (0, 255, 255), 3)
            
            # Add text overlays
            cv2.putText(debug_image, f'Distance: {distance:.3f} m', 
                       (200, 700), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 0), 2)
            cv2.putText(debug_image, f'Confidence: {confidence:.3f}', 
                       (200, 650), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 0), 2)
            cv2.putText(debug_image, f'Offset X: {offset_x:.1f} || Offset Y: {offset_y:.1f}', 
                       (100, 600), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 0), 2)
            cv2.putText(debug_image, 'YOLO Gate Detection', 
                       (50, 50), cv2.FONT_HERSHEY_COMPLEX, 1, (255, 0, 0), 2)
            
            # Publish debug image
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, 'bgr8')
            self.debug_image_publisher.publish(debug_msg)


def main(args=None):
    rclpy.init(args=args)
    node = YOLOGateDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()