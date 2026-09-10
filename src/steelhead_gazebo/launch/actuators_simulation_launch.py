import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    ld = LaunchDescription()
    actuators_command = Node(
        name='actuators_command_simulation',
        namespace='/steelhead/controls',
        package='steelhead_gazebo',
        executable='actuators_command_simulation',
        output='screen',
    )
    
    ld.add_action(actuators_command)

    return ld