"""
Launch file for the path follow simulation.
Places the AUV above the orange pathmarker and follows the orange path.
Based on gate_navigation_test_launch.py structure.
"""
import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level'
    )

    log_level = LaunchConfiguration('log_level')

    ld = LaunchDescription([log_level_arg])

    pid_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('steelhead_pid_controller'), 'launch', 'steelhead_pid_controller_launch.py')
        )
    )

    hover_at_depth = Node(
        package='steelhead_controls',
        executable='hover_at_depth',
        namespace='/steelhead',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'depth': -1.2},
            {'adjust_yaw': False},
        ],
    )

    path_follow_route = Node(
        package='steelhead_controls',
        executable='path_follow_route.py',
        name='path_follow_route',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    pathmarker_detector = Node(
        package='steelhead_controls',
        executable='pathmarker_detector.py',
        name='pathmarker_detector',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('steelhead_gazebo'), 'launch', 'gazebo_launch.py')
        ),
        launch_arguments={'world': 'path_follow.world'}.items()
    )

    config = os.path.join(
        get_package_share_directory('steelhead_localization'),
        'config',
        'state_estimator_config.yaml'
    )

    state_estimator = Node(
        name='state_estimator',
        namespace='/steelhead/controls/ukf',
        package='robot_localization',
        executable='ukf_node',
        output='screen',
        parameters=[config, {'use_sim_time': True}]
    )

    state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('steelhead_controls'), 'launch', 'state_publisher_launch.py')
        )
    )

    thrust_allocator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('steelhead_controls'), 'launch', 'thrust_allocator_launch.py')
        )
    )

    underwater_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            get_package_share_directory('steelhead_gazebo') + '/launch/underwater_camera_launch.py'
        )
    )

    # trajectory_generator removed - path_follow_route handles navigation

    pkg_share = get_package_share_directory('steelhead_gazebo')
    rviz_config_file = os.path.join(pkg_share, 'config', 'rviz_config.rviz')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    ld.add_action(gazebo)
    ld.add_action(rviz)
    ld.add_action(thrust_allocator)
    ld.add_action(state_publisher)
    ld.add_action(underwater_camera)
    ld.add_action(state_estimator)
    ld.add_action(hover_at_depth)
    ld.add_action(pathmarker_detector)
    ld.add_action(path_follow_route)
    ld.add_action(pid_controller)
    # trajectory_generator removed

    return ld
