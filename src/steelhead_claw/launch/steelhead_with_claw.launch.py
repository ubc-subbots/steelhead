#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()

    # Launch arguments
    declare_world_arg = DeclareLaunchArgument(
        'world',
        default_value='competition.world',
        description='Gazebo world file to load'
    )

    declare_enable_claw_arg = DeclareLaunchArgument(
        'enable_claw',
        default_value='true',
        description='Enable the claw system'
    )

    # Get launch configuration
    world = LaunchConfiguration('world')
    enable_claw = LaunchConfiguration('enable_claw')

    # Start Gazebo with the specified world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('triton_gazebo'), 'launch', 'gazebo_launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    # Start thrust allocator for AUV movement
    thrust_allocator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('triton_controls'), 'launch', 'thrust_allocator_launch.py')
        )
    )

    # Start keyboard teleop for manual control
    keyboard_teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('triton_teleop'), 'launch', 'keyboard_teleop_launch.py')
        )
    )

    # Start claw control system
    claw_system = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('steelhead_claw'), 'launch', 'claw_only.launch.py')
        )
    )

    # Optional: Add a claw teleop node for keyboard control of the claw
    claw_teleop = Node(
        package='steelhead_claw',
        executable='claw_teleop',
        name='claw_teleop',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Add launch arguments
    ld.add_action(declare_world_arg)
    ld.add_action(declare_enable_claw_arg)

    # Add all system launches
    ld.add_action(gazebo)
    ld.add_action(thrust_allocator)
    ld.add_action(keyboard_teleop)
    ld.add_action(claw_system)
    
    # Uncomment the next line if you create a claw_teleop executable
    # ld.add_action(claw_teleop)

    return ld