
import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    ld = LaunchDescription()

    hover_at_depth = Node(
        package='steelhead_controls', 
        executable='hover_at_depth',
        parameters=[{'depth': 1.0, 'adjust_yaw': False}],
        namespace="steelhead"
    )

    ld.add_action(hover_at_depth)

    return ld