import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    ld = LaunchDescription()

    qualifying_task = Node(
        name='qualifying_task',
        namespace='/triton/controls',
        package='triton_controls',
        executable='qualifying_task',
        output='screen',
    )

    ld.add_action(qualifying_task)

    return ld