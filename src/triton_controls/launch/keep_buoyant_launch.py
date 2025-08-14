
import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    ld = LaunchDescription()
    keep_buoyant = Node(
        package='triton_controls', 
        executable='keep_buoyant',
        output='screen', 
    )

    ld.add_action(keep_buoyant)

    return ld