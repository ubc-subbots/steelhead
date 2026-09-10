
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    ld = LaunchDescription()

    trajectory_generator = Node(
        name='trajectory_generator',
        namespace='/steelhead/controls',
        package='steelhead_controls',
        executable='trajectory_generator',
        output='screen',
    )

    ld.add_action(trajectory_generator)

    return ld