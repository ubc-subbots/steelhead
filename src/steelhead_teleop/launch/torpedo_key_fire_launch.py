from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    torpedo_key_fire = Node(
        name='torpedo_key_fire',
        package='steelhead_teleop',
        executable='torpedo_key_fire',
        output='screen',
        parameters=[{
            'service_name': '/steelhead/controls/actuators_command',
            'command': 'torpedo',
            'cooldown_s': 0.0,
        }],
    )

    ld.add_action(torpedo_key_fire)

    return ld
