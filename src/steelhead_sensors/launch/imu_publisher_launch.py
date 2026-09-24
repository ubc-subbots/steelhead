
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    ld = LaunchDescription()

    bno085_imu_publisher = Node(
        name='bno085_imu_publisher',
        namespace='/steelhead/controls',
        package='steelhead_controls',
        executable='bno085_imu_publisher.py',
        output='screen',
    )

    ld.add_action(bno085_imu_publisher)

    return ld