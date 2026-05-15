import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():

    config_path = os.path.join(
        get_package_share_directory('steelhead_localization'),
        'config',
        'steelhead_gazebo_vins_config.yaml'
    )

    vins_path = os.path.join(
        get_package_share_directory('steelhead_localization'),
        'config',
        '..'
    )

    support_path = os.path.join(
        get_package_share_directory('steelhead_localization'),
        'config',
        'support_files'
    )

    vins_estimator_node = Node(
        package='steelhead_localization',
        namespace='vins_estimator',
        executable='vins_estimator',
        name='vins_estimator',
        output='screen',
        parameters=[{
            'config_file': config_path,
            'vins_folder': vins_path,
            'use_sim_time': True
        }]
    )

    feature_tracker_node = Node(
        package='steelhead_localization',
        namespace='feature_tracker',
        executable='feature_tracker',
        name='feature_tracker',
        output='screen',
        parameters=[{
            'config_file': config_path,
            'vins_folder': vins_path,
            'use_sim_time': True
        }]
    )

    pose_graph_node = Node(
        package='steelhead_localization',
        executable='pose_graph',
        name='pose_graph',
        namespace='pose_graph',
        output='screen',
        parameters=[{
            'config_file': config_path,
            'support_file': support_path,
            'visualization_shift_x': 0,
            'visualization_shift_y': 0,
            'skip_cnt': 0,
            'skip_dis': 0.0
        }]
    )

    return LaunchDescription([
        vins_estimator_node,
        feature_tracker_node,
        pose_graph_node
    ])