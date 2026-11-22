import launch
from launch_ros.actions import Node


def generate_launch_description():

    ld = launch.LaunchDescription()

    path_marker_detector = Node(
        package="steelhead_vision",
        executable="path_marker_detector.py",
        name="path_marker_detector",
        parameters=[
            {"input_image_topic": "/steelhead/drivers/bottom_camera/image_raw"},
            {"detection_topic": "/steelhead/path_marker/detection"},
            {"debug_image_topic": "/steelhead/path_marker/debug_image"},
            {"hsv_lower": [5, 100, 100]},
            {"hsv_upper": [25, 255, 255]},
            {"min_area_fraction": 0.01},
        ],
        output="screen",
    )

    ld.add_action(path_marker_detector)

    return ld
