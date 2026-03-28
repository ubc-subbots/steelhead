from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('steelhead_mission_planner'),
        'config',
        'mission_tree.yaml'
    )

    return LaunchDescription([
        Node(
            package='steelhead_mission_planner',
            executable='mission_planner',
            name='mission_planner',
            output='screen',
            parameters=[config]
        )
    ])
