from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    ld = LaunchDescription()

    depth_publisher = Node(
        name='ms5837_depth_publisher',
        namespace='steelhead',
        package='steelhead_controls',
        executable='ms5837_depth_publisher.py',
        output='screen',
        parameters=[{
            'port': '/dev/depth',
            'baud': 115200,
        }],
    )

    ld.add_action(depth_publisher)

    return ld
