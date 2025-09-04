import os
from ament_index_python.packages import get_package_share_directory
import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions.node import Node
from launch_ros.descriptions import ComposableNode
import yaml


def generate_launch_description():

    ld = launch.LaunchDescription()

    with open(os.path.join(
            get_package_share_directory('steelhead_object_recognition'),
            'config',
            'realsense.yaml',), 'r') as f:
        realsense_config = yaml.load(f, Loader=yaml.Loader)

    realsense = Node(
        name='realsense',
        namespace='/steelhead',
        package='realsense2_camera',
        parameters=[realsense_config],
        executable='realsense2_camera_node'
    )

    config = os.path.join(
        get_package_share_directory('steelhead_object_recognition'),
        'config',
        'tiny_yolov4.yaml'
    )

    object_recognizer = ComposableNode(
        name='object_recognizer',
        namespace='/steelhead',
        package='steelhead_object_recognition',
        parameters=[config],
        plugin='steelhead_object_recognition::ObjectRecognizer',
        remappings=[('/steelhead/object_recognizer/in', '/steelhead/color/image_raw')]
    )

    object_recognizer_container = ComposableNodeContainer(
        name='object_recognizer_container',
        namespace='/steelheadimage_underwater',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            object_recognizer
        ],
        output='screen'
    )

    ld.add_action(realsense)
    ld.add_action(object_recognizer_container)

    return ld
