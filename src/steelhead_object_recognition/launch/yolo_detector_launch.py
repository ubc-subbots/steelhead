import os

import launch
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():

    ld = launch.LaunchDescription()

    weights_path = os.path.join(
        get_package_share_directory("steelhead_object_recognition"),
        "config",
        "ffc_rs_26.pt",
    )

    yolo_detector = Node(
        package="steelhead_object_recognition",
        executable="yolo_detector.py",
        name="yolo_detector",
        parameters=[
            {"weights_path": weights_path},
            {"confidence_threshold": 0.4},
            {"inference_interval": 1.0},
        ],
        output="screen",
    )

    ld.add_action(yolo_detector)

    return ld
