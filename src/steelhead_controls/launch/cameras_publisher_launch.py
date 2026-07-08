import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    ld = LaunchDescription()

    camera1 = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        namespace="/camera1",
        parameters=[
            os.path.join(
                get_package_share_directory("steelhead_bringup"),
                "config",
                "camera1.yaml",
            )
        ],
        remappings=[
            ("/camera1/image_raw", "/steelhead/drivers/bottom_camera/image_raw"),
            ("/camera1/camera_info", "/steelhead/drivers/bottom_camera/camera_info"),
        ],
    )

    ld.add_action(camera1)

    camera2 = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        namespace="/camera2",
        parameters=[
            os.path.join(
                get_package_share_directory("steelhead_bringup"),
                "config",
                "camera2.yaml",
            )
        ],
        remappings=[
            ("/camera2/image_raw", "/steelhead/drivers/front_camera/image_raw"),
            ("/camera2/camera_info", "/steelhead/drivers/front_camera/camera_info"),
        ],
    )

    ld.add_action(camera2)

    return ld
