from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='steelhead_claw',
            executable='claw_controller',
            name='claw_controller',
            output='screen'
        ),
        Node(
            package='steelhead_claw',
            executable='claw_hardware_interface',
            name='claw_hardware_interface',
            output='screen'
        ),
    ])