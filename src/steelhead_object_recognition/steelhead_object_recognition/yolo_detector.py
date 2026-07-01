#!/usr/bin/env python3

import os
os.environ["CUDA_VISIBLE_DEVICES"] = ""  # Forces everything to CPU

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from steelhead_interfaces.msg import DetectionBox, DetectionBoxArray
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
import cv2
import numpy as np
from pathlib import Path
import time


try:
    from ultralytics import YOLO
    ULTRALYTICS_AVAILABLE = True
except ImportError as e:
    ULTRALYTICS_AVAILABLE = False
    import sys
    print(f"DEBUG: ultralytics import failed with error: {e}. Try 'pip install ultralytics'")


class YOLODetector(Node):
    """
    YOLO detector node that runs inference on camera images and publishes annotated results.
    """

    def __init__(self):
        super().__init__('yolo_detector')
        
        self.bridge = CvBridge()
        self.last_inference_time = 0
        self.inference_interval = 1.0  # Run inference every 1 second
        
        # Declare parameters. 
        default_weights_path = os.path.join(
            get_package_share_directory('steelhead_object_recognition'),
            'config', 'competition.pt'
        )
        self.declare_parameter('weights_path', default_weights_path)  # Can be overridden
        self.declare_parameter('confidence_threshold', 0.1)
        self.declare_parameter('inference_interval', 1.0)
        
        weights_path = self.get_parameter('weights_path').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.inference_interval = self.get_parameter('inference_interval').value
        
        # Load YOLO model
        self.model = None
        if ULTRALYTICS_AVAILABLE:
            try:
                self.get_logger().info(f'Loading YOLO model from: {weights_path}')
                self.model = YOLO(weights_path).to('cpu')
                self.get_logger().info('YOLO model loaded successfully')
            except Exception as e:
                self.get_logger().error(f'Failed to load YOLO model: {e}')
                self.get_logger().error('Running in pass-through mode (raw image republish)')
        else:
            self.get_logger().warning('ultralytics package not available. Running in pass-through mode.')
        
        # Subscribers and publishers - create these AFTER model loading attempt
        # so the node doesn't crash even if model loading fails
        self.image_subscriber = self.create_subscription(
            Image,
            '/steelhead/drivers/front_camera/image_raw',
            self.image_callback,
            10
        )
        
        self.annotated_publisher = self.create_publisher(
            Image,
            '/steelhead/yolo_detector/annotated_image',
            10
        )

        self.detections_publisher = self.create_publisher(
            DetectionBoxArray,
            '/steelhead/yolo_detector/detections',
            10
        )

        self.get_logger().info('YOLO Detector node initialized')
        self.frame_count = 0

    def image_callback(self, msg: Image):
        """
        Callback for incoming camera images.
        Runs YOLO inference and publishes annotated image.
        """
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Throttle inference to specified interval
            current_time = time.time()
            if current_time - self.last_inference_time < self.inference_interval:
                return
            
            self.last_inference_time = current_time
            self.frame_count += 1

            # Pixel detections
            detection_array = DetectionBoxArray()
            detection_array.header = msg.header

            # If model is not loaded, just republish the original image
            if self.model is None:
                annotated_image = cv_image
                # self.get_logger().info(f'Frame {self.frame_count}: Model not loaded, republishing raw image')
            else:
                # Run YOLO inference
                results = self.model(cv_image, conf=self.confidence_threshold, verbose=False, device='cpu')

                # self.get_logger().info(f'Frame {self.frame_count}: {results}')  # Debug: print raw results to console
                
                # Annotate image with detections
                annotated_image = cv_image.copy()
                
                if results and len(results) > 0:
                    result = results[0]
                    detections = result.boxes
                    
                    self.get_logger().info(f'Frame {self.frame_count}: {len(detections)} detections')
                    
                    # Draw bounding boxes
                    for box in detections:
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        conf = float(box.conf[0])
                        cls_id = int(box.cls[0])
                        cls_name = result.names[cls_id]

                        # Record machine-readable detection (xy = top-left corner, pixels)
                        detection_box = DetectionBox()
                        detection_box.class_id = cls_id
                        detection_box.label = cls_name
                        detection_box.confidence = conf
                        detection_box.x = float(x1)
                        detection_box.y = float(y1)
                        detection_box.width = float(x2 - x1)
                        detection_box.height = float(y2 - y1)
                        detection_array.boxes.append(detection_box)

                        # Draw rectangle
                        cv2.rectangle(annotated_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        
                        # Put label
                        label = f'{cls_name} {conf:.2f}'
                        label_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                        cv2.rectangle(
                            annotated_image,
                            (x1, y1 - label_size[1] - 4),
                            (x1 + label_size[0], y1),
                            (0, 255, 0),
                            -1
                        )
                        cv2.putText(
                            annotated_image,
                            label,
                            (x1, y1 - 2),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            (0, 0, 0),
                            1
                        )
            
            # Publish machine-readable detections (empty array when nothing detected)
            self.detections_publisher.publish(detection_array)

            # Convert back to ROS image and publish
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding='bgr8')
            annotated_msg.header = msg.header
            self.annotated_publisher.publish(annotated_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error in image callback: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = YOLODetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
