import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('steelhead_gazebo'), 'launch', 'gazebo_launch.py')
        ),
        launch_arguments={'world': 'prequalification.world'}.items()
    )

    thrust_allocator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('steelhead_controls'), 'launch', 'thrust_allocator_launch.py')
        )
    )

    rviz_config_file = os.path.join(
        get_package_share_directory('steelhead_bringup'), 'config', 'prequalification_task.rviz')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}], 
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

    # vins_odometry = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory('steelhead_localization'), 'launch', 'vins_odometry_launch.py')
    #     )
    # )

    state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('steelhead_controls'), 'launch', 'state_publisher_launch.py')
        )
    )

    # there's some small mis matches with our physical model and simulation which results in a bunch of errors
    # in the terminal despite it working as expected. this a bandaid fix that does nothing but stops the false errors
    base_link_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_tf_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_link::base_link'],
        parameters=[{'use_sim_time': True}]
    )

    pid_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('steelhead_pid_controller'), 'launch', 'steelhead_pid_controller_launch.py')
        )
    )

    waypoint_marker = Node(
        package='steelhead_controls', 
        executable='waypoint_marker',
        output='screen', 
        parameters=[{'use_sim_time': True}]
    )

    predetermined_route = Node(
        package='steelhead_controls', 
        executable='predetermined_route.py',
        output='screen', 
        parameters=[{'use_sim_time': True}]
    )
    
    ld.add_action(gazebo)
    ld.add_action(rviz)
    ld.add_action(thrust_allocator)
    ld.add_action(state_publisher)
    # ld.add_action(underwater_camera) # the underwater camera simulator isn't that good and is very taxing on performance, so i'm disabling it for now
    ld.add_action(state_estimator)
    ld.add_action(base_link_tf_publisher)
    ld.add_action(pid_controller)
    ld.add_action(waypoint_marker)
    ld.add_action(predetermined_route)

    return ld