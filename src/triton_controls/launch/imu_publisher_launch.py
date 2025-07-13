import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    ld = LaunchDescription()

    bno085_imu_publisher = Node(
        name='bno085_imu_publisher',
        namespace='/triton/controls',
        package='triton_controls',
        executable='bno085_imu_publisher.py',
        output='screen',
    )

    ld.add_action(bno085_imu_publisher)

    return ld