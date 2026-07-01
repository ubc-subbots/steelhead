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
        launch_arguments={'world': 'competition_task_gate.world'}.items()
    )

    thrust_allocator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('steelhead_controls'), 'launch', 'thrust_allocator_launch.py')
        )
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

    rviz_config_file = os.path.join(
        get_package_share_directory('steelhead_bringup'), 'config', 'gazebo_test.rviz')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}], 
    )

    automation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('steelhead_tasks'), 'launch', 'automated_competition_gate_launch.py')
        )
    )

    yolo_detector = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('steelhead_object_recognition'), 'launch', 'yolo_detector_launch.py')
        )
    ) 
    
    ld.add_action(gazebo)
    ld.add_action(thrust_allocator)
    ld.add_action(hover_script)
    ld.add_action(pid_controller)
    ld.add_action(rviz)
    ld.add_action(automation)
    ld.add_action(yolo_detector)

    return ld
