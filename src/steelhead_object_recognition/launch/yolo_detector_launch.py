import os

import launch
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():

    ld = launch.LaunchDescription()

    front_camera_weights = os.path.join(
        get_package_share_directory("steelhead_object_recognition"),
        "config",
        "ffc_rs_26.pt",
    )

    yolo_detector_front = Node(
        package="steelhead_object_recognition",
        executable="yolo_detector.py",
        name="yolo_detector_front",
        parameters=[
            {"weights_path": front_camera_weights},
            {"confidence_threshold": 0.4},
            {"inference_interval": 1.0},
            {"input_image_topic": "/steelhead/drivers/front_camera/image_raw"},
            {"annotated_image_topic": "/steelhead/yolo_detector/front/annotated_image"},
            {"detections_topic": "/steelhead/yolo_detector/front/detections"},
        ],
        output="screen",
    )

    down_camera_weights = os.path.join(
        get_package_share_directory("steelhead_object_recognition"),
        "config",
        "dfc_rs_26.pt",
    )

    yolo_detector_bottom = Node(
        package="steelhead_object_recognition",
        executable="yolo_detector.py",
        name="yolo_detector_bottom",
        parameters=[
            {"weights_path": down_camera_weights},
            {"confidence_threshold": 0.4},
            {"inference_interval": 1.0},
            {"input_image_topic": "/steelhead/drivers/bottom_camera/image_raw"},
            {
                "annotated_image_topic": "/steelhead/yolo_detector/bottom/annotated_image"
            },
            {"detections_topic": "/steelhead/yolo_detector/bottom/detections"},
        ],
        output="screen",
    )

    ld.add_action(yolo_detector_front)
    ld.add_action(yolo_detector_bottom)

    return ld
