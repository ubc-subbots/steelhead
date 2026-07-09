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
        launch_arguments={'world': 'competition_task_slaloms.world'}.items()
    )

    thrust_allocator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('steelhead_controls'), 'launch', 'thrust_allocator_launch.py')
        )
    )

    nav_node = Node(
        package='steelhead_gazebo',
        executable='nav.py',
        name='nav',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    keyboard_teleop = Node(
        name='keyboard_teleop',
        namespace='/steelhead/teleop',
        package='steelhead_teleop',
        executable='keyboard_teleop',
        output='screen',
        parameters=[{'publish_topic': '/steelhead/controls/hover_adjust'}],
    )

    hover_script = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('steelhead_controls'), 'launch', 'hover_at_depth_launch.py')
        )
    )

    pid_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('steelhead_pid_controller'), 'launch', 'steelhead_pid_controller_launch.py')
        )
    )

    rqt_reconfigure_node = Node(
        name='rqt_reconfigure',
        package='rqt_reconfigure',
        executable='rqt_reconfigure',
        output='screen',
        arguments=['/steelhead/controls/steelhead_pid_controller']
    )

    delayed_rqt = TimerAction(
        period=15.0,
        actions=[rqt_reconfigure_node]
    )

    ld.add_action(gazebo)
    ld.add_action(thrust_allocator)
    ld.add_action(nav_node)
    ld.add_action(keyboard_teleop)
    ld.add_action(hover_script)
    ld.add_action(pid_controller)

    return ld