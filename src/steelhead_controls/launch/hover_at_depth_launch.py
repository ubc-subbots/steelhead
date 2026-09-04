
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    ld = LaunchDescription()

    hover_at_depth = Node(
        package="steelhead_controls",
        executable="hover_at_depth",
        parameters=[{"depth": 1.0, "hold_yaw": False}],
        namespace="steelhead",
    )

    ld.add_action(hover_at_depth)

    return ld
