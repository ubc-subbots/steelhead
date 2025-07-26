#!/usr/bin/env python3

import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Package directories
    triton_gazebo_path = get_package_share_directory('triton_gazebo')
    steelhead_claw_path = get_package_share_directory('steelhead_claw')
    gazebo_ros_path = get_package_share_directory('gazebo_ros')
    
    # URDF file path - using the actual triton_auv model
    urdf_file = os.path.join(triton_gazebo_path, 'models', 'triton_auv', 'model.urdf')
    
    # World file path
    world_file = os.path.join(steelhead_claw_path, 'worlds', 'underwater_world.world')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_simulator = LaunchConfiguration('use_simulator')
    headless = LaunchConfiguration('headless')
    world = LaunchConfiguration('world')
    
    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')
    
    declare_use_simulator_cmd = DeclareLaunchArgument(
        'use_simulator',
        default_value='True',
        description='Whether to start the simulator')
    
    declare_simulator_cmd = DeclareLaunchArgument(
        'headless',
        default_value='False',
        description='Whether to execute gzclient)')
    
    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=world_file,
        description='Full path to world model file to load')
    
    # Get URDF via xacro
    robot_description_content = Command([
        'xacro ', urdf_file
    ])
    robot_description = {'robot_description': robot_description_content}
    
    # Start Gazebo server
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_path, 'launch', 'gzserver.launch.py')),
        condition=IfCondition(use_simulator),
        launch_arguments={'world': world,
                         'verbose': 'true'}.items())
    
    # Start Gazebo client    
    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_path, 'launch', 'gzclient.launch.py')),
        condition=IfCondition(PythonExpression([use_simulator, ' and not ', headless])))
    
    # Robot state publisher
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}])
    
    # Spawn the robot in Gazebo
    spawn_entity_cmd = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-entity', 'steelhead',
                   '-topic', 'robot_description',
                   '-x', '0', '-y', '0', '-z', '1'],
        output='screen')
    
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
    
    # Joint state publisher (if needed)
    start_joint_state_publisher_cmd = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[robot_description, {'use_sim_time': use_sim_time}])
    
    # Create the launch description and populate
    ld = LaunchDescription()
    
    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_world_cmd)
    
    # Add any conditioned actions
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    
    # Add the actions to launch all nodes
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(spawn_entity_cmd)
    ld.add_action(start_claw_controller_cmd)
    ld.add_action(start_claw_hardware_cmd)
    ld.add_action(start_joint_state_publisher_cmd)
    
    return ld