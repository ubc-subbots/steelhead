import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    pkg_share = get_package_share_directory('spiderfish_gazebo')

    train_yolo = Node(
        package="spiderfish_gazebo",
        executable='train_yolo.py',
        name='train_yolo'
    )

    ld.add_action(train_yolo)
    return ld