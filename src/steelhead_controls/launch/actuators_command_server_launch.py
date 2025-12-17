import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('steelhead_controls'),
        'config',
        'actuators_config.yaml'
    )

    actuators_command = Node(
        name='actuators_command',
        namespace='/steelhead/controls',
        package='steelhead_controls',
        executable='actuators_command',
        output='screen',
        parameters=[config],
    )
    
    ld.add_action(actuators_command)

    return ld