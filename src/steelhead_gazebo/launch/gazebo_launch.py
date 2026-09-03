import launch_ros.actions
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    ld = LaunchDescription()

    world_arg = DeclareLaunchArgument(
            'world',
            default_value='empty.world',
            description='Gazebo world file'
    )

    headless_arg = DeclareLaunchArgument(
            'headless',
            default_value='false',
            description="Set to 'true' to run gazebo headless"
    )

    steelhead_gazebo_dir = get_package_share_directory('steelhead_gazebo')

    # IGN_GAZEBO_RESOURCE_PATH is used in newer Ignitions/Gazebo Sim
    grp = 'GZ_SIM_RESOURCE_PATH'
    add_resource_path = SetEnvironmentVariable(
        name=grp, 
        value=[
            EnvironmentVariable(grp, default_value=''), 
            os.pathsep + os.path.join(steelhead_gazebo_dir, 'gazebo'),
            os.pathsep + os.path.join(steelhead_gazebo_dir, 'gazebo', 'models')
        ]
    )

    # Arguments for gz_sim
    gz_args = PythonExpression([
        '"-r -v 4 "',
        ' + "-s " if "', LaunchConfiguration('headless'), '" == "true" else ""',
        ' + " worlds/" + "', LaunchConfiguration('world'), '"'
    ])

    gazebo_sim = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 
            'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': gz_args}.items()
    )

    ld.add_action(world_arg)
    ld.add_action(headless_arg)
    ld.add_action(add_resource_path)
    ld.add_action(gazebo_sim)
    bridge = launch_ros.actions.Node(package="ros_gz_bridge", executable="parameter_bridge", arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"], output="screen")
    ld.add_action(bridge)
    return ld
