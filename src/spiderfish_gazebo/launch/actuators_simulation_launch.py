import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('spiderfish_controls'),
        'config',
        'actuators_config.yaml'
    )

    actuators_command = Node(
        name='actuators_command_simulation',
        namespace='/spiderfish/controls',
        package='spiderfish_gazebo',
        executable='actuators_command_simulation',
        output='screen',
        parameters=[config],
    )
    
    ld.add_action(actuators_command)

    return ld