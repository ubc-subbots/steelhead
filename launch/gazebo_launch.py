#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package directory
    pkg_steelhead = get_package_share_directory('gazebo')

    # Start Gazebo server and client
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
    )

    # Delay the spawning to ensure Gazebo is ready
    spawn_robot = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='steelhead_urdf',
                executable='spawn_steelhead_robot.py',
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        gz_sim,
        spawn_robot,
    ])
