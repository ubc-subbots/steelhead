import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    ld = LaunchDescription()
    hover_underwater = Node(
        package='triton_controls', 
        executable='hover_underwater',
        output='screen', 
    )

    ld.add_action(hover_underwater)

    return ld