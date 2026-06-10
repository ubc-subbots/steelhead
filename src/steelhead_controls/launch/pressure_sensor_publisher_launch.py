import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    ld = LaunchDescription()

    pressure_publisher = Node(
        name='ms5837_pressure_sensor_publisher',
        namespace='steelhead',
        package='steelhead_controls',
        executable='ms5837_pressure_sensor_publisher.py',
        output='screen',
    )

    ld.add_action(pressure_publisher)

    return ld