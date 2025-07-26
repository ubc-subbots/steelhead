#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')
    
    # Start claw controller
    start_claw_controller_cmd = Node(
        package='steelhead_claw',
        executable='claw_controller',
        name='claw_controller',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}])
    
    # Start claw hardware interface (or simulation interface)
    start_claw_hardware_cmd = Node(
        package='steelhead_claw',
        executable='claw_hardware_interface',
        name='claw_hardware_interface',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}])
    
    # Create the launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    
    # Add nodes
    ld.add_action(start_claw_controller_cmd)
    ld.add_action(start_claw_hardware_cmd)
    
    return ld