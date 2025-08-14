from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()

    # Serial communication with AUV hardware
    serial = ComposableNodeContainer(
        name='serial_subscriber_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='triton_controls',
                plugin='triton_controls::SerialSubscriber',
                name='serial_subscriber'),
        ],
        output='both',
    ) 

    # IMU launch for compass and orientation data
    imu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(os.path.dirname(__file__), 'imu_launch.py')
        )
    )

    # Thrust allocator configuration for real robot
    ta_config = os.path.join(
        get_package_share_directory('triton_controls'),
        'config',
        'thruster_config.yaml'
    )

    # Thrust allocator for motor control - sends to real thrusters via serial
    thrust_allocator = Node(
        name='thrust_allocator',
        namespace='/triton/controls',
        package='triton_controls',
        executable='thrust_allocator',
        output='screen',
        parameters=[ta_config],
        remappings=[
            ('/triton/controls/signals', '/motor_control')  # Routes to serial
        ]
    )

    # Direct IMU controller - same as simulation!
    direct_controller = Node(
        package='triton_controls',
        executable='direct_imu_controller',
        name='direct_imu_controller',
        output='screen'
        # Note: NO use_sim_time parameter for real robot
    )

    # Launch everything
    ld.add_action(serial)      # Start serial communication
    ld.add_action(imu)         # Start IMU
    ld.add_action(thrust_allocator)  # Start thrust allocator
    ld.add_action(direct_controller) # Start our controller

    return ld