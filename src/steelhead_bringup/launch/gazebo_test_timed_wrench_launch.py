import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('steelhead_gazebo'), 'launch', 'gazebo_launch.py')
        ),
        launch_arguments={'world': 'competition.world'}.items()
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

    # Pipeline manager + sequence manager + component container, configured to
    # run two TimedWrench maneuvers back to back (forward drive, then style
    # spin). No teleop node here: it would publish to the same
    # controls/hover_adjust topic the tasks use and fight them for control.
    pipeline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('steelhead_pipeline'), 'launch', 'pipeline_launch.py')
        ),
        launch_arguments={'sequence': 'timed_wrench_demo_sequence.yaml'}.items()
    )

    # A TimedWrench task starts its duration clock the moment it is loaded, so
    # hold the pipeline back until gazebo has spun up and the AUV is in the
    # water, otherwise the first maneuver burns its time against a paused sim.
    delayed_pipeline = TimerAction(
        period=15.0,
        actions=[pipeline]
    )

    ld.add_action(gazebo)
    ld.add_action(thrust_allocator)
    ld.add_action(hover_script)
    ld.add_action(pid_controller)
    ld.add_action(delayed_pipeline)

    return ld
