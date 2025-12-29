import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    ld = LaunchDescription()

    odom_to_pose_node = Node(
        name='odom_to_pose',
        namespace='/steelhead/controls',
        package='steelhead_controls',
        executable='odom_to_pose',
        output='screen',
    )

    ld.add_action(odom_to_pose_node)

    return ld
